"""短秤台记录的轴重还原与磅单绑定 (short-platform axle reconstruction).

一辆车组通过长度 ``L`` 的短秤台时，设备只在 *进轴/出轴位置* 记录台面总重。
本模块把这样一条记录绑定到一张 *具体磅单*，按该票票面历史组合（目录产品的
稳定身份）选择当时的车轴布局，精确还原每根轴的正整数载重：

* 车轴布局 :class:`AxleLayout`：相对前轴的整数位置 ``d``，首轴 ``0``，向车尾
  严格递增；每车至多 :data:`MAX_AXLES` 根轴。
* 前轴走到 ``x`` 时，轴 ``i`` 在台面上当且仅当 ``0 <= x - d_i < L``（半开区间，
  右端点出轴）。进轴位置为 ``d_i``，出轴位置为 ``d_i + L``；同一位置可以同时
  发生一次进轴与一次出轴，记录的是全部同位动作完成后的台面总重。
* 还原在有理数域精确求解 0/1 线性方程组，再要求每根轴载重为 *正整数*，并核对
  末端归零与磅单毛重。

任何漏点、多余点、重复点、读数矛盾、末端不归零、非正轴重、毛重不符或历史
车组没有布局的记录都带 *可定位的问题码*，``valid=False``，且
:func:`weighbridge.routing.shortest_route_for_record` 会拒绝把它用于线路复核。
复核只能使用绑票还原出的原车组轴距与轴重——另一张同毛重票不可代替。

容量：每车至多 16 轴；纯标准库（``fractions.Fraction``）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from fractions import Fraction
from typing import Any

from .catalog import Product
from .warehouse import Warehouse, WarehouseError

MAX_AXLES = 16

# ---------------------------------------------------------------- 问题码
TICKET_NOT_FOUND = "TICKET_NOT_FOUND"              # 绑定的磅单不存在
HISTORY_LAYOUT_MISSING = "HISTORY_LAYOUT_MISSING"  # 票面历史车组没有登记轴布局
BAD_PLATFORM_LENGTH = "BAD_PLATFORM_LENGTH"        # 台长 L 非正整数
BAD_READING = "BAD_READING"                        # 读数不是非负整数
DUPLICATE_POINT = "DUPLICATE_POINT"                # 同一位置记了两次
MISSING_POINT = "MISSING_POINT"                    # 该进/出轴位置没有读数（漏点）
UNEXPECTED_POINT = "UNEXPECTED_POINT"              # 读数位置不是任何进/出轴位置
END_NOT_ZERO = "END_NOT_ZERO"                      # 末轴出后台面未归零
INCONSISTENT_READINGS = "INCONSISTENT_READINGS"    # 方程组无解，读数互相矛盾
UNDERDETERMINED = "UNDERDETERMINED"                # 方程组秩不足（理论兜底）
NON_INTEGER_AXLE = "NON_INTEGER_AXLE"              # 解出的轴重不是整数
NON_POSITIVE_AXLE = "NON_POSITIVE_AXLE"            # 解出的轴重不是正整数
GROSS_MISMATCH = "GROSS_MISMATCH"                  # 轴重和与磅单毛重不一致


class AxleError(ValueError):
    """轴布局或秤台记录的输入结构错误。"""


def _as_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise AxleError(f"{name} must be an integer, got {value!r}")
    return value


# ---------------------------------------------------------------- 数据结构
@dataclass(frozen=True)
class AxleLayout:
    """某历史车组（稳定身份）的车轴布局：相对前轴的整数位置。"""

    tractor: str
    trailer: str | None
    positions: tuple[int, ...]

    def __post_init__(self) -> None:
        if not str(self.tractor).strip():
            raise AxleError("layout tractor id must be non-empty")
        pos = tuple(_as_int(d, "axle position") for d in self.positions)
        if not 1 <= len(pos) <= MAX_AXLES:
            raise AxleError(f"axle count must be in 1..{MAX_AXLES}, got {len(pos)}")
        if pos[0] != 0:
            raise AxleError(f"first axle position must be 0, got {pos[0]}")
        if any(pos[i] >= pos[i + 1] for i in range(len(pos) - 1)):
            raise AxleError(f"axle positions must be strictly increasing: {pos}")
        if any(d < 0 for d in pos):
            raise AxleError(f"axle positions must be non-negative: {pos}")
        object.__setattr__(self, "positions", pos)
        object.__setattr__(self, "tractor", str(self.tractor).strip())
        if self.trailer is not None:
            object.__setattr__(self, "trailer", str(self.trailer).strip())

    @property
    def axle_count(self) -> int:
        return len(self.positions)

    def identity(self) -> tuple[str, str | None]:
        return (self.tractor, self.trailer)

    def to_dict(self) -> dict:
        return {
            "tractor": self.tractor,
            "trailer": self.trailer,
            "positions": list(self.positions),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "AxleLayout":
        return cls(
            tractor=data["tractor"],
            trailer=data.get("trailer"),
            positions=tuple(data["positions"]),
        )


class AxleRegistry:
    """稳定身份 -> 当时车轴布局的登记表；同身份只允许一份不冲突布局。"""

    def __init__(self) -> None:
        self._layouts: dict[tuple[str, str | None], AxleLayout] = {}

    def register(self, layout: AxleLayout) -> AxleLayout:
        key = layout.identity()
        old = self._layouts.get(key)
        if old is not None and old.positions != layout.positions:
            raise AxleError(
                f"layout for {key} already registered as {old.positions}, "
                f"refusing conflict {layout.positions}"
            )
        self._layouts[key] = layout
        return layout

    def layout_for_identity(self, identity: tuple[str, str | None]) -> AxleLayout:
        try:
            return self._layouts[identity]
        except KeyError:
            raise AxleError(f"no axle layout registered for historical group {identity}")

    def layout_for_product(self, product: Product) -> AxleLayout:
        return self.layout_for_identity(product.stable_identity())

    def identities(self) -> list[tuple[str, str | None]]:
        return sorted(self._layouts, key=lambda x: (x[0], x[1] or ""))

    def __len__(self) -> int:
        return len(self._layouts)

    def to_dict(self) -> dict:
        return {
            "Kind": "axle_registry",
            "Version": 1,
            "layouts": [self._layouts[k].to_dict() for k in self.identities()],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @classmethod
    def from_dict(cls, data: dict) -> "AxleRegistry":
        if data.get("Kind") != "axle_registry":
            raise AxleError("not an axle_registry document")
        if int(data.get("Version", 0)) != 1:
            raise AxleError(f"unsupported axle_registry Version {data.get('Version')!r}")
        reg = cls()
        for raw in data["layouts"]:
            reg.register(AxleLayout.from_dict(raw))
        return reg

    @classmethod
    def from_json(cls, text: str) -> "AxleRegistry":
        try:
            return cls.from_dict(json.loads(text))
        except json.JSONDecodeError as exc:
            raise AxleError(f"invalid registry json: {exc}") from exc


@dataclass(frozen=True)
class PlatformRecord:
    """绑定到具体 ``ticket_id`` 的短秤台原始记录。

    ``readings`` 为 ``(位置, 台面总重)`` 序列，顺序随意；重复位置会在还原时
    被定位为 :data:`DUPLICATE_POINT`。
    """

    ticket_id: str
    platform_length: int
    readings: tuple[tuple[int, int], ...]

    def __post_init__(self) -> None:
        if not str(self.ticket_id).strip():
            raise AxleError("ticket_id must be non-empty")
        object.__setattr__(self, "ticket_id", str(self.ticket_id).strip())
        object.__setattr__(
            self, "platform_length", _as_int(self.platform_length, "platform_length")
        )
        rs: list[tuple[int, int]] = []
        for p, w in self.readings:
            rs.append((_as_int(p, "reading position"), _as_int(w, "reading weight")))
        object.__setattr__(self, "readings", tuple(rs))

    @classmethod
    def from_dict(cls, data: dict) -> "PlatformRecord":
        return cls(
            ticket_id=data["ticket_id"],
            platform_length=data["platform_length"],
            readings=tuple((int(p), int(w)) for p, w in data["readings"]),
        )

    def to_dict(self) -> dict:
        return {
            "ticket_id": self.ticket_id,
            "platform_length": self.platform_length,
            "readings": [list(rw) for rw in self.readings],
        }


@dataclass(frozen=True)
class ReconstructionIssue:
    """一条可定位的记录问题；``location`` 给出位置/轴号或 ``"-"``。"""

    code: str
    location: str
    detail: str

    def to_dict(self) -> dict:
        return {"code": self.code, "location": self.location, "detail": self.detail}


@dataclass(frozen=True)
class EventAudit:
    """某个进/出轴位置上，全部同位动作完成后的台面事实。"""

    position: int
    entering: tuple[int, ...]   # 进轴的轴序号（0 起）
    exiting: tuple[int, ...]    # 出轴的轴序号
    on_platform: tuple[int, ...]
    recorded: int | None        # 设备读数（缺记/无法解析时为 None）
    computed: int | None        # 由还原轴重算出的台面总重

    def to_dict(self) -> dict:
        return {
            "position": self.position,
            "entering": list(self.entering),
            "exiting": list(self.exiting),
            "on_platform": list(self.on_platform),
            "recorded": self.recorded,
            "computed": self.computed,
        }


@dataclass(frozen=True)
class Reconstruction:
    """一条秤台记录对一张磅单的还原结果；无效记录 ``valid=False``。"""

    ticket_id: str
    gross: int | None
    identity: tuple[str, str | None] | None
    layout: AxleLayout | None
    platform_length: int | None
    axle_loads: tuple[int, ...] | None
    valid: bool
    issues: tuple[ReconstructionIssue, ...]
    events: tuple[EventAudit, ...]

    @property
    def axle_count(self) -> int:
        return 0 if self.layout is None else self.layout.axle_count

    def require_valid(self) -> tuple[AxleLayout, tuple[int, ...]]:
        """线路复核入口：无效记录直接拒绝，绝不带着问题进入复核。"""
        if not self.valid or self.layout is None or self.axle_loads is None:
            codes = [i.code for i in self.issues]
            raise AxleError(
                f"record bound to ticket {self.ticket_id} is not usable for route "
                f"review: {codes}"
            )
        return self.layout, self.axle_loads

    def to_dict(self) -> dict:
        return {
            "ticket_id": self.ticket_id,
            "gross": self.gross,
            "identity": None if self.identity is None else list(self.identity),
            "layout": None if self.layout is None else self.layout.to_dict(),
            "platform_length": self.platform_length,
            "axle_loads": None if self.axle_loads is None else list(self.axle_loads),
            "valid": self.valid,
            "issues": [i.to_dict() for i in self.issues],
            "events": [e.to_dict() for e in self.events],
        }


# ---------------------------------------------------------------- 事件与模拟
def expected_event_positions(layout: AxleLayout, platform_length: int) -> list[int]:
    """全部进轴位置 ``d_i`` 与出轴位置 ``d_i + L`` 的严格升序并集。"""
    positions: set[int] = set()
    for d in layout.positions:
        positions.add(d)
        positions.add(d + platform_length)
    return sorted(positions)


def expected_events(layout: AxleLayout, platform_length: int) -> list[EventAudit]:
    """不依赖读数的事件骨架：每个位置谁进、谁出、动作后谁在台面上。"""
    events: list[EventAudit] = []
    for x in expected_event_positions(layout, platform_length):
        entering = tuple(i for i, d in enumerate(layout.positions) if d == x)
        exiting = tuple(
            i for i, d in enumerate(layout.positions) if d + platform_length == x
        )
        on = tuple(
            i for i, d in enumerate(layout.positions) if d <= x < d + platform_length
        )
        events.append(
            EventAudit(
                position=x, entering=entering, exiting=exiting,
                on_platform=on, recorded=None, computed=None,
            )
        )
    return events


def simulate_readings(
    layout: AxleLayout, axle_loads: Sequence[int], platform_length: int
) -> list[tuple[int, int]]:
    """按题面半开区间模型合成一份 *必然合规* 的设备记录（供合成档案/测试）。"""
    if len(axle_loads) != layout.axle_count:
        raise AxleError("axle_loads width does not match layout")
    loads = [_as_int(a, "axle load") for a in axle_loads]
    if any(a <= 0 for a in loads):
        raise AxleError("simulated axle loads must be positive integers")
    if platform_length <= 0:
        raise AxleError("platform_length must be a positive integer")
    out: list[tuple[int, int]] = []
    for ev in expected_events(layout, platform_length):
        total = sum(loads[i] for i in ev.on_platform)
        out.append((ev.position, total))
    return out


# ---------------------------------------------------------------- 精确求解
def _solve_01_system(
    rows: Sequence[Sequence[int]], rhs: Sequence[int]
) -> tuple[list[Fraction] | None, bool, int, tuple[int, ...]]:
    """高斯-若尔当消元（精确有理数）。

    返回 ``(解, 是否矛盾, 秩, 矛盾涉及的原始行号)``；秩不足且不矛盾时解为
    ``None``。每行消元中携带其由哪些原始方程线性组合而成的标签，一旦出现
    ``0 = b != 0``，标签即可把矛盾定位回具体读数位置。
    """
    n_cols = len(rows[0])
    mat = [
        [Fraction(v) for v in row] + [Fraction(b)]
        for row, b in zip(rows, rhs)
    ]
    labels: list[set[int]] = [{i} for i in range(len(mat))]
    rank = 0
    pivot_cols: list[int] = []
    for col in range(n_cols):
        pivot = next((r for r in range(rank, len(mat)) if mat[r][col] != 0), None)
        if pivot is None:
            continue
        mat[rank], mat[pivot] = mat[pivot], mat[rank]
        labels[rank], labels[pivot] = labels[pivot], labels[rank]
        pv = mat[rank][col]
        mat[rank] = [v / pv for v in mat[rank]]
        for r in range(len(mat)):
            if r != rank and mat[r][col] != 0:
                factor = mat[r][col]
                mat[r] = [a - factor * b for a, b in zip(mat[r], mat[rank])]
                labels[r] |= labels[rank]
        pivot_cols.append(col)
        rank += 1
    for r in range(rank, len(mat)):
        if mat[r][n_cols] != 0:
            return None, True, rank, tuple(sorted(labels[r]))
    if rank < n_cols:
        return None, False, rank, ()
    x = [Fraction(0)] * n_cols
    for r, col in enumerate(pivot_cols):
        x[col] = mat[r][n_cols]
    return x, False, rank, ()


def _issue(code: str, location: str, detail: str) -> ReconstructionIssue:
    return ReconstructionIssue(code=code, location=location, detail=detail)


def _reconstruct_core(
    record: PlatformRecord,
    layout: AxleLayout,
    gross: int,
) -> Reconstruction:
    issues: list[ReconstructionIssue] = []
    identity = layout.identity()
    L = record.platform_length

    if L <= 0:
        return Reconstruction(
            ticket_id=record.ticket_id, gross=gross, identity=identity, layout=layout,
            platform_length=L, axle_loads=None, valid=False,
            issues=(_issue(BAD_PLATFORM_LENGTH, "-", f"platform length {L} <= 0"),),
            events=(),
        )

    skeleton = expected_events(layout, L)
    expected_positions = [ev.position for ev in skeleton]
    last_position = expected_positions[-1]

    # 原始读数解析与重复点定位。
    recorded: dict[int, int] = {}
    for p, w in record.readings:
        if w < 0:
            issues.append(_issue(BAD_READING, f"x={p}", f"negative platform weight {w}"))
            continue
        if p in recorded:
            issues.append(
                _issue(DUPLICATE_POINT, f"x={p}", f"position recorded more than once")
            )
            continue
        recorded[p] = w

    missing = [p for p in expected_positions if p not in recorded]
    extras = sorted(p for p in recorded if p not in set(expected_positions))
    for p in missing:
        issues.append(_issue(MISSING_POINT, f"x={p}", "expected entry/exit position has no reading"))
    for p in extras:
        issues.append(
            _issue(UNEXPECTED_POINT, f"x={p}", "no axle enters or exits the platform here")
        )

    events: list[EventAudit] = []
    for ev in skeleton:
        events.append(
            EventAudit(
                position=ev.position, entering=ev.entering, exiting=ev.exiting,
                on_platform=ev.on_platform,
                recorded=recorded.get(ev.position), computed=None,
            )
        )

    # 末端归零是显式物理核对项。
    end_recorded = recorded.get(last_position)
    if end_recorded is not None and end_recorded != 0:
        issues.append(
            _issue(
                END_NOT_ZERO, f"x={last_position}",
                f"all axles have exited but platform reads {end_recorded}, expected 0",
            )
        )

    structural = {MISSING_POINT, UNEXPECTED_POINT, DUPLICATE_POINT, BAD_READING}
    if any(i.code in structural for i in issues):
        return Reconstruction(
            ticket_id=record.ticket_id, gross=gross, identity=identity, layout=layout,
            platform_length=L, axle_loads=None, valid=False,
            issues=tuple(issues), events=tuple(events),
        )

    # 点位齐全：建立 0/1 在台方程组并精确求解。
    rows = [
        [1 if i in ev.on_platform else 0 for i in range(layout.axle_count)]
        for ev in events
    ]
    rhs = [recorded[ev.position] for ev in events]
    solution, inconsistent, rank, conflict_rows = _solve_01_system(rows, rhs)
    if inconsistent:
        located = ", ".join(f"x={events[r].position}" for r in conflict_rows) or "-"
        issues.append(
            _issue(
                INCONSISTENT_READINGS,
                located,
                "no axle-load vector satisfies all recorded platform totals",
            )
        )
        return Reconstruction(
            ticket_id=record.ticket_id, gross=gross, identity=identity, layout=layout,
            platform_length=L, axle_loads=None, valid=False,
            issues=tuple(issues), events=tuple(events),
        )
    if solution is None:
        issues.append(
            _issue(UNDERDETERMINED, "-", f"equation rank {rank} < {layout.axle_count} axles")
        )
        return Reconstruction(
            ticket_id=record.ticket_id, gross=gross, identity=identity, layout=layout,
            platform_length=L, axle_loads=None, valid=False,
            issues=tuple(issues), events=tuple(events),
        )

    axle_issues: list[ReconstructionIssue] = []
    loads: list[int] = []
    for i, value in enumerate(solution):
        if value.denominator != 1:
            axle_issues.append(
                _issue(NON_INTEGER_AXLE, f"axle#{i} d={layout.positions[i]}",
                       f"solved load {value} is not an integer")
            )
        elif value <= 0:
            axle_issues.append(
                _issue(NON_POSITIVE_AXLE, f"axle#{i} d={layout.positions[i]}",
                       f"solved load {value.numerator} is not a positive integer")
            )
        loads.append(value.numerator if value.denominator == 1 else 0)

    # 用解回填每个事件的计算台面总重，形成完整审计轨迹。
    audited: list[EventAudit] = []
    for ev in events:
        computed = sum(
            solution[i] for i in ev.on_platform
        )
        audited.append(
            EventAudit(
                position=ev.position, entering=ev.entering, exiting=ev.exiting,
                on_platform=ev.on_platform, recorded=ev.recorded,
                computed=computed.numerator if computed.denominator == 1 else None,
            )
        )

    total = sum(loads)
    if total != gross:
        issues.append(
            _issue(
                GROSS_MISMATCH, "-",
                f"sum of axle loads {total} != ticket gross {gross}",
            )
        )

    all_issues = tuple(issues + axle_issues)
    return Reconstruction(
        ticket_id=record.ticket_id, gross=gross, identity=identity, layout=layout,
        platform_length=L,
        axle_loads=tuple(loads) if not axle_issues else None,
        valid=not all_issues,
        issues=all_issues, events=tuple(audited),
    )


# ---------------------------------------------------------------- 对外入口
def reconstruct_record(
    record: PlatformRecord,
    warehouse: Warehouse,
    registry: AxleRegistry,
) -> Reconstruction:
    """把短秤台记录绑定到具体磅单并还原轴重。

    流程：取磅单 -> 按票面 ``catalog_seq`` 取 *历史组合产品*（目录或外部快照，
    绝不从当前挂接猜）-> 按稳定身份查当时的车轴布局 -> 精确求解与全部核对。
    记录层面的任何问题都收敛为带问题码的无效 :class:`Reconstruction`，不抛异常。
    """
    try:
        ticket = warehouse.ticket(record.ticket_id)
    except WarehouseError as exc:
        return Reconstruction(
            ticket_id=record.ticket_id, gross=None, identity=None, layout=None,
            platform_length=record.platform_length, axle_loads=None, valid=False,
            issues=(_issue(TICKET_NOT_FOUND, "-", str(exc)),), events=(),
        )

    derivation = warehouse.derive_ticket(ticket.ticket_id)
    product = derivation.product
    identity = product.stable_identity()
    try:
        layout = registry.layout_for_identity(identity)
    except AxleError as exc:
        return Reconstruction(
            ticket_id=record.ticket_id, gross=ticket.gross, identity=identity,
            layout=None, platform_length=record.platform_length,
            axle_loads=None, valid=False,
            issues=(_issue(HISTORY_LAYOUT_MISSING, "-", str(exc)),), events=(),
        )
    return _reconstruct_core(record, layout, ticket.gross)
