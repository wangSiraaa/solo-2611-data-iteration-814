"""线路档案、桥面影响线峰值与最短可行线路 (route archive & bridge review).

档案是一张有向图：节点为字符串，每条 :class:`RoadSegment` 有正整数行程时间；
桥面路段另外持有一条 *仅用于此档案* 的有理数分段线性影响线
:class:`InfluenceLine` 与反应上限 ``limit``。

语义（严格按题面简化荷载模型）
------------------------------
* 影响线 ``f``：折点横坐标严格递增，定义区间两端取值为零，区间外为零；至多
  :data:`MAX_BREAKPOINTS` 个折点，坐标与取值均为精确有理数。
* 车组轴距 ``d_i``（相对前轴，首轴 0）、轴重 ``w_i``（正整数），前轴位于
  ``x`` 时桥面反应 ``R(x) = Σ_i w_i · f(x - d_i)``。
* 每条折线是若干仿射函数之和，其最大值一定落在折点（经平移后的折点并集）上，
  故只在折点集合上求值即得整个 *连续* 过桥过程的精确最大值；全程用
  ``fractions.Fraction``，不引入浮点。
* 路段可通行当且仅当精确峰值 ``<= limit``；等于上限即放行。不同路段可用不同
  曲线，绝不退化成 "只看最重单轴"。
* 最短时间可行线路用带字典序标签的 Dijkstra：时间相同时取路段编号序列
  字典序最小者。无路可选时报告从起点在 *底层图* 上可达的节点/路段范围，以及
  挡住出口的桥面证据（峰值、上限、超限量、见证位置、各轴贡献）。

容量：每车至多 16 轴（由 ``axle`` 模块强制），每条曲线至多 40 个折点。
"""

from __future__ import annotations

import heapq
import json
from dataclasses import dataclass
from fractions import Fraction
from typing import Any, Iterable, Sequence

from .axle import AxleLayout, Reconstruction

MAX_BREAKPOINTS = 40

ROUTE_FEASIBLE = "FEASIBLE"        # 找到最短时间可行线路
ROUTE_BLOCKED = "BLOCKED"          # 终点在底层可达范围内，但被桥面上限挡住
ROUTE_UNREACHABLE = "UNREACHABLE"  # 终点不在底层可达范围内（无路径）


class RoutingError(ValueError):
    """线路档案或查询的输入结构错误。"""


# ---------------------------------------------------------------- 有理数
def _fraction(value: Any, name: str) -> Fraction:
    if isinstance(value, bool):
        raise RoutingError(f"{name} must be a rational, got bool")
    if isinstance(value, Fraction):
        return value
    if isinstance(value, int):
        return Fraction(value)
    if isinstance(value, str):
        try:
            return Fraction(value)
        except (ValueError, ZeroDivisionError) as exc:
            raise RoutingError(f"{name} has invalid rational text {value!r}") from exc
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return Fraction(int(value[0]), int(value[1]))
    raise RoutingError(f"{name} must be int/Fraction/[num,den]/rational str, got {value!r}")


def _json_rational(value: Fraction) -> int | str:
    """Fraction 的可交换序列化：整数直接出 int，否则出约分分数字符串。"""
    return value.numerator if value.denominator == 1 else f"{value.numerator}/{value.denominator}"


# ---------------------------------------------------------------- 影响线
@dataclass(frozen=True)
class InfluenceLine:
    """分段线性影响线；折点序列含区间两端的零点。"""

    points: tuple[tuple[Fraction, Fraction], ...]

    def __post_init__(self) -> None:
        pts = tuple((_fraction(x, "breakpoint x"), _fraction(y, "breakpoint y"))
                    for x, y in self.points)
        if not 2 <= len(pts) <= MAX_BREAKPOINTS:
            raise RoutingError(
                f"influence line needs 2..{MAX_BREAKPOINTS} breakpoints, got {len(pts)}"
            )
        for i in range(len(pts) - 1):
            if pts[i][0] >= pts[i + 1][0]:
                raise RoutingError("breakpoint x coordinates must be strictly increasing")
        if pts[0][1] != 0 or pts[-1][1] != 0:
            raise RoutingError("influence line must be zero at both interval ends")
        object.__setattr__(self, "points", pts)

    @property
    def support(self) -> tuple[Fraction, Fraction]:
        return self.points[0][0], self.points[-1][0]

    def value_at(self, z: Fraction) -> Fraction:
        """区间外为零；折点上取折点值，区间内线性插值，全部精确。"""
        pts = self.points
        if z < pts[0][0] or z > pts[-1][0]:
            return Fraction(0)
        lo, hi = 0, len(pts) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if pts[mid][0] <= z:
                lo = mid
            else:
                hi = mid
        x0, y0 = pts[lo]
        x1, y1 = pts[hi]
        if x1 == x0:
            return y0
        t = (z - x0) / (x1 - x0)
        return y0 + t * (y1 - y0)

    def to_dict(self) -> dict:
        return {
            "points": [[_json_rational(x), _json_rational(y)] for x, y in self.points]
        }

    @classmethod
    def from_dict(cls, data: dict) -> "InfluenceLine":
        return cls(points=tuple((x, y) for x, y in data["points"]))


@dataclass(frozen=True)
class BridgeSpec:
    """桥面路段的专属曲线与反应上限（仅用于所属档案）。"""

    influence_line: InfluenceLine
    limit: Fraction

    def __post_init__(self) -> None:
        object.__setattr__(self, "limit", _fraction(self.limit, "bridge limit"))

    def to_dict(self) -> dict:
        return {"influence_line": self.influence_line.to_dict(),
                "limit": _json_rational(self.limit)}

    @classmethod
    def from_dict(cls, data: dict) -> "BridgeSpec":
        return cls(
            influence_line=InfluenceLine.from_dict(data["influence_line"]),
            limit=_fraction(data["limit"], "bridge limit"),
        )


@dataclass(frozen=True)
class RoadSegment:
    """有向路段；``bridge`` 为 ``None`` 即普通路段，无荷载约束。"""

    segment_id: int
    source: str
    target: str
    travel_time: int
    bridge: BridgeSpec | None = None

    def __post_init__(self) -> None:
        if isinstance(self.segment_id, bool) or not isinstance(self.segment_id, int):
            raise RoutingError("segment_id must be an integer")
        if not str(self.source).strip() or not str(self.target).strip():
            raise RoutingError("segment endpoints must be non-empty")
        if isinstance(self.travel_time, bool) or not isinstance(self.travel_time, int):
            raise RoutingError("travel_time must be an integer")
        if self.travel_time <= 0:
            raise RoutingError(f"travel_time must be a positive integer, got {self.travel_time}")

    @property
    def is_bridge(self) -> bool:
        return self.bridge is not None

    def to_dict(self) -> dict:
        return {
            "segment_id": self.segment_id,
            "source": self.source,
            "target": self.target,
            "travel_time": self.travel_time,
            "bridge": None if self.bridge is None else self.bridge.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "RoadSegment":
        bridge = data.get("bridge")
        return cls(
            segment_id=int(data["segment_id"]),
            source=str(data["source"]),
            target=str(data["target"]),
            travel_time=int(data["travel_time"]),
            bridge=None if bridge is None else BridgeSpec.from_dict(bridge),
        )


class RouteArchive:
    """有向线路档案；路段编号唯一，可并行、可自环。"""

    def __init__(self, segments: Iterable[RoadSegment] = ()) -> None:
        self._segments: dict[int, RoadSegment] = {}
        self._out: dict[str, list[RoadSegment]] = {}
        for seg in segments:
            self.add_segment(seg)

    def add_segment(self, segment: RoadSegment) -> RoadSegment:
        if segment.segment_id in self._segments:
            raise RoutingError(f"duplicate segment_id {segment.segment_id}")
        self._segments[segment.segment_id] = segment
        self._out.setdefault(segment.source, []).append(segment)
        self._out.setdefault(segment.target, self._out.get(segment.target, []))
        return segment

    def segment(self, segment_id: int) -> RoadSegment:
        try:
            return self._segments[segment_id]
        except KeyError:
            raise RoutingError(f"unknown segment {segment_id}")

    def segments(self) -> list[RoadSegment]:
        return [self._segments[k] for k in sorted(self._segments)]

    def nodes(self) -> list[str]:
        return sorted(self._out)

    def outgoing(self, node: str) -> list[RoadSegment]:
        return sorted(self._out.get(node, ()), key=lambda s: s.segment_id)

    def to_dict(self) -> dict:
        return {
            "Kind": "route_archive",
            "Version": 1,
            "segments": [s.to_dict() for s in self.segments()],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @classmethod
    def from_dict(cls, data: dict) -> "RouteArchive":
        if data.get("Kind") != "route_archive":
            raise RoutingError("not a route_archive document")
        if int(data.get("Version", 0)) != 1:
            raise RoutingError(f"unsupported route_archive Version {data.get('Version')!r}")
        return cls(RoadSegment.from_dict(raw) for raw in data["segments"])

    @classmethod
    def from_json(cls, text: str) -> "RouteArchive":
        try:
            return cls.from_dict(json.loads(text))
        except json.JSONDecodeError as exc:
            raise RoutingError(f"invalid archive json: {exc}") from exc


# ---------------------------------------------------------------- 桥面复核
@dataclass(frozen=True)
class AxleContribution:
    """见证位置处单根轴的贡献 ``w_i · f(x - d_i)``。"""

    axle_index: int
    position: int
    load: int
    influence: Fraction
    contribution: Fraction

    def to_dict(self) -> dict:
        return {
            "axle_index": self.axle_index,
            "axle_position": self.position,
            "load": self.load,
            "influence": _json_rational(self.influence),
            "contribution": _json_rational(self.contribution),
        }


@dataclass(frozen=True)
class BridgeReview:
    """单个桥面路段对指定车组的完整精确复核。"""

    segment_id: int
    peak: Fraction
    witness_x: Fraction
    limit: Fraction
    feasible: bool
    excess: Fraction
    contributions: tuple[AxleContribution, ...]

    def to_dict(self) -> dict:
        return {
            "segment_id": self.segment_id,
            "peak": _json_rational(self.peak),
            "witness_x": _json_rational(self.witness_x),
            "limit": _json_rational(self.limit),
            "feasible": self.feasible,
            "excess": _json_rational(self.excess),
            "contributions": [c.to_dict() for c in self.contributions],
        }


def _candidate_positions(
    line: InfluenceLine, layout: AxleLayout
) -> list[Fraction]:
    """全部候选前轴位置：每个折点经每根轴距平移后的并集，升序去重。

    只保留至少一根轴落在影响线支撑区间内的位置；区间外的轴贡献恒为零，
    不参与最大值竞争。
    """
    lo, hi = line.support
    xs: set[Fraction] = set()
    for bx, _ in line.points:
        for d in layout.positions:
            x = bx + d
            if any(lo <= x - d2 <= hi for d2 in layout.positions):
                xs.add(x)
    return sorted(xs)


def review_bridge(
    segment: RoadSegment,
    layout: AxleLayout,
    axle_loads: Sequence[int],
) -> BridgeReview:
    """求连续过桥过程的精确峰值、见证位置与各轴贡献，并对上限放行判断。"""
    if segment.bridge is None:
        raise RoutingError(f"segment {segment.segment_id} is not a bridge")
    if len(axle_loads) != layout.axle_count:
        raise RoutingError("axle_loads width does not match layout")
    loads = [int(w) for w in axle_loads]
    if any(w <= 0 for w in loads):
        raise RoutingError("axle loads must be positive integers")
    line = segment.bridge.influence_line
    candidates = _candidate_positions(line, layout)
    if not candidates:
        # 退化的零支撑曲线（两端重合会被构造器拒绝），理论不可达；保守按零反应。
        candidates = [line.support[0]]

    best_peak: Fraction | None = None
    best_x: Fraction | None = candidates[0]
    best_values: list[Fraction] = []
    for x in candidates:
        values = [line.value_at(x - d) for d in layout.positions]
        peak = sum(
            (Fraction(w) * v for w, v in zip(loads, values)), Fraction(0)
        )
        if best_peak is None or peak > best_peak:
            best_peak, best_x, best_values = peak, x, values
    assert best_peak is not None
    contributions = tuple(
        AxleContribution(
            axle_index=i,
            position=layout.positions[i],
            load=loads[i],
            influence=best_values[i],
            contribution=Fraction(loads[i]) * best_values[i],
        )
        for i in range(layout.axle_count)
    )
    limit = segment.bridge.limit
    return BridgeReview(
        segment_id=segment.segment_id,
        peak=best_peak,
        witness_x=best_x,
        limit=limit,
        feasible=best_peak <= limit,
        excess=max(Fraction(0), best_peak - limit),
        contributions=contributions,
    )


# ---------------------------------------------------------------- 无路证据
@dataclass(frozen=True)
class BlockingEvidence:
    """从可达区域出发的桥面路段为何挡住出口。"""

    segment_id: int
    source: str
    target: str
    review: BridgeReview

    def to_dict(self) -> dict:
        return {
            "segment_id": self.segment_id,
            "source": self.source,
            "target": self.target,
            "review": self.review.to_dict(),
        }


@dataclass(frozen=True)
class RouteResult:
    """一次可行线路查询的完整结论（可行 / 被桥挡住 / 底层不可达）。"""

    status: str
    source: str
    target: str
    time: int | None
    segment_path: tuple[int, ...] | None
    node_path: tuple[str, ...] | None
    bridge_reviews: tuple[BridgeReview, ...]
    reachable_nodes: tuple[str, ...]
    reachable_edges: tuple[tuple[int, str, str], ...]
    blocking: tuple[BlockingEvidence, ...]

    def to_dict(self) -> dict:
        return {
            "status": self.status,
            "source": self.source,
            "target": self.target,
            "time": self.time,
            "segment_path": None if self.segment_path is None else list(self.segment_path),
            "node_path": None if self.node_path is None else list(self.node_path),
            "bridge_reviews": [r.to_dict() for r in self.bridge_reviews],
            "reachable_nodes": list(self.reachable_nodes),
            "reachable_edges": [list(e) for e in self.reachable_edges],
            "blocking": [b.to_dict() for b in self.blocking],
        }


def _underlying_reach(
    archive: RouteArchive, source: str
) -> tuple[set[str], set[int]]:
    """底层有向图（忽略桥面约束）上从起点可达的节点与路段。"""
    seen_nodes = {source}
    seen_edges: set[int] = set()
    stack = [source]
    while stack:
        node = stack.pop()
        for seg in archive.outgoing(node):
            seen_edges.add(seg.segment_id)
            if seg.target not in seen_nodes:
                seen_nodes.add(seg.target)
                stack.append(seg.target)
    return seen_nodes, seen_edges


def shortest_route(
    archive: RouteArchive,
    source: str,
    target: str,
    layout: AxleLayout,
    axle_loads: Sequence[int],
) -> RouteResult:
    """最短时间可行线路；时间相同取路段编号序列字典序最小。

    标签为 ``(累计时间, 路段编号元组, 节点路径元组)``，Dijkstra 的非负边权与
    标签全序共同保证弹出终点时即最优；同时间的更优字典序路径会替换旧标签。
    """
    all_nodes = set(archive.nodes())
    if source not in all_nodes or target not in all_nodes:
        raise RoutingError(f"endpoints {source!r}->{target!r} must both exist in archive")

    review_cache: dict[int, BridgeReview] = {}

    def review_of(seg: RoadSegment) -> BridgeReview:
        if seg.segment_id not in review_cache:
            review_cache[seg.segment_id] = review_bridge(seg, layout, axle_loads)
        return review_cache[seg.segment_id]

    def passable(seg: RoadSegment) -> bool:
        return seg.bridge is None or review_of(seg).feasible

    # 标签：(time, segment id tuple, node tuple)
    labels: dict[str, tuple[int, tuple[int, ...], tuple[str, ...]]] = {
        source: (0, (), (source,))
    }
    heap: list[tuple[int, tuple[int, ...], str]] = [(0, (), source)]
    while heap:
        time, path, node = heapq.heappop(heap)
        cur = labels.get(node)
        if cur is None or cur[0] != time or cur[1] != path:
            continue  # 过期标签
        if node == target:
            break
        for seg in archive.outgoing(node):
            if not passable(seg):
                continue
            ntime = time + seg.travel_time
            npath = path + (seg.segment_id,)
            old = labels.get(seg.target)
            candidate = (ntime, npath, cur[2] + (seg.target,))
            if old is None or (candidate[0], candidate[1]) < (old[0], old[1]):
                labels[seg.target] = candidate
                heapq.heappush(heap, (ntime, npath, seg.target))

    reachable_nodes, reachable_edges = _underlying_reach(archive, source)

    if target in labels:
        time, path, node_path = labels[target]
        used = set(path)
        reviews = tuple(
            review_cache[sid] for sid in sorted(used)
            if archive.segment(sid).is_bridge
        )
        return RouteResult(
            status=ROUTE_FEASIBLE, source=source, target=target,
            time=time, segment_path=path, node_path=node_path,
            bridge_reviews=reviews,
            reachable_nodes=tuple(sorted(reachable_nodes)),
            reachable_edges=tuple(
                sorted((sid, archive.segment(sid).source, archive.segment(sid).target)
                       for sid in reachable_edges)
            ),
            blocking=(),
        )

    # 无路：枚举从可达区域出发、因桥面超限而不可通行的出口，附精确证据。
    blocking: list[BlockingEvidence] = []
    for node in sorted(reachable_nodes):
        for seg in archive.outgoing(node):
            if seg.is_bridge and not review_of(seg).feasible:
                blocking.append(
                    BlockingEvidence(
                        segment_id=seg.segment_id, source=seg.source,
                        target=seg.target, review=review_of(seg),
                    )
                )
    blocking.sort(key=lambda b: b.segment_id)
    status = ROUTE_BLOCKED if target in reachable_nodes else ROUTE_UNREACHABLE
    return RouteResult(
        status=status, source=source, target=target,
        time=None, segment_path=None, node_path=None,
        bridge_reviews=tuple(review_cache.values()),
        reachable_nodes=tuple(sorted(reachable_nodes)),
        reachable_edges=tuple(
            sorted((sid, archive.segment(sid).source, archive.segment(sid).target)
                   for sid in reachable_edges)
        ),
        blocking=tuple(blocking),
    )


def shortest_route_for_record(
    archive: RouteArchive,
    source: str,
    target: str,
    reconstruction: Reconstruction,
) -> RouteResult:
    """线路复核的对外入口：只接受绑定具体磅单且核对通过的还原记录。

    同毛重的别张票无法代替——车组轴距与轴重全部来自 ``reconstruction`` 自身
    绑定的那张票；无效记录在此被拒绝，不产生任何线路结论。
    """
    layout, axle_loads = reconstruction.require_valid()
    return shortest_route(archive, source, target, layout, axle_loads)
