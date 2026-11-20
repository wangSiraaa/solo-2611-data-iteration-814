"""影响线峰值与最短可行线路的端到端测试。

运行::

    python3 -m unittest tests.test_routing -v
"""

from __future__ import annotations

import itertools
import random
import unittest
from fractions import Fraction

from weighbridge import (
    AxleError,
    AxleLayout,
    AxleRegistry,
    BridgeSpec,
    Catalog,
    InfluenceLine,
    MAX_BREAKPOINTS,
    PlatformRecord,
    ROUTE_BLOCKED,
    ROUTE_FEASIBLE,
    ROUTE_UNREACHABLE,
    RoadSegment,
    RouteArchive,
    RoutingError,
    Warehouse,
    reconstruct_record,
    review_bridge,
    shortest_route,
    shortest_route_for_record,
)
from weighbridge.routing import _fraction


def triangle_line() -> InfluenceLine:
    return InfluenceLine(((0, 0), (2, 1), (4, 0)))


def layout_2axle() -> AxleLayout:
    return AxleLayout("T1", "R1", (0, 2))


class InfluenceLineTests(unittest.TestCase):
    def test_endpoints_must_be_zero_and_sorted(self):
        with self.assertRaises(RoutingError):
            InfluenceLine(((0, 1), (2, 0)))            # 左端非零
        with self.assertRaises(RoutingError):
            InfluenceLine(((0, 0), (4, 1)))            # 右端非零
        with self.assertRaises(RoutingError):
            InfluenceLine(((0, 0), (4, 0), (2, 0)))    # 横坐标逆序
        InfluenceLine(((0, 0), (4, 0)))                # 两点曲线合法
        with self.assertRaises(RoutingError):
            InfluenceLine(((0, 0),))                   # 单点不合法
        with self.assertRaises(RoutingError):
            InfluenceLine(((0, 0),) * (MAX_BREAKPOINTS + 1))

    def test_value_at_pieces_and_outside(self):
        line = triangle_line()
        self.assertEqual(line.value_at(Fraction(-1)), 0)
        self.assertEqual(line.value_at(Fraction(5)), 0)
        self.assertEqual(line.value_at(Fraction(0)), 0)
        self.assertEqual(line.value_at(Fraction(2)), 1)
        self.assertEqual(line.value_at(Fraction(4)), 0)
        self.assertEqual(line.value_at(Fraction(1)), Fraction(1, 2))
        self.assertEqual(line.value_at(Fraction(3)), Fraction(1, 2))

    def test_rational_breakpoints_exact(self):
        line = InfluenceLine(((0, 0), (Fraction(1, 3), 1), (Fraction(2, 3), 0)))
        self.assertEqual(line.value_at(Fraction(1, 6)), Fraction(1, 2))
        self.assertEqual(_fraction("2/7", "x"), Fraction(2, 7))
        with self.assertRaises(RoutingError):
            _fraction(1.5, "float rejected")


class BridgeReviewTests(unittest.TestCase):
    def test_textbook_triangle_peak_3_5(self):
        seg = RoadSegment(1, "S", "E", 2, BridgeSpec(triangle_line(), Fraction(4)))
        rev = review_bridge(seg, layout_2axle(), (3, 5))
        self.assertEqual(rev.peak, 5)
        self.assertEqual(rev.witness_x, 4)
        self.assertFalse(rev.feasible)
        self.assertEqual(rev.excess, 1)
        # 见证位置 x=4：后轴在 f(2)=1 上贡献 5；前轴已出支撑区间贡献 0。
        self.assertEqual(rev.contributions[0].contribution, 0)
        self.assertEqual(rev.contributions[1].contribution, 5)

    def test_limit_equality_passes(self):
        seg = RoadSegment(1, "S", "E", 2, BridgeSpec(triangle_line(), Fraction(5)))
        rev = review_bridge(seg, layout_2axle(), (3, 5))
        self.assertTrue(rev.feasible)
        self.assertEqual(rev.excess, 0)

    def test_4_4_passes_limit_4_where_3_5_fails(self):
        seg = RoadSegment(1, "S", "E", 2, BridgeSpec(triangle_line(), Fraction(4)))
        rev = review_bridge(seg, layout_2axle(), (4, 4))
        self.assertEqual(rev.peak, 4)
        self.assertTrue(rev.feasible)
        self.assertEqual(rev.witness_x, 2)

    def test_two_axles_both_contribute_on_plateau_curve(self):
        # 平台曲线 (0,0)(2,1)(4,1)(6,0)；x=4 时两轴分别落在 f(4)=1、f(2)=1。
        plateau = InfluenceLine(((0, 0), (2, 1), (4, 1), (6, 0)))
        seg = RoadSegment(7, "S", "E", 2, BridgeSpec(plateau, Fraction(8)))
        rev = review_bridge(seg, layout_2axle(), (3, 5))
        self.assertEqual(rev.peak, 8)
        self.assertEqual(rev.witness_x, 4)
        nonzero = [c for c in rev.contributions if c.contribution > 0]
        self.assertEqual(len(nonzero), 2)
        self.assertEqual({c.axle_index for c in nonzero}, {0, 1})
        self.assertEqual(sum((c.contribution for c in rev.contributions), Fraction(0)), 8)
        # 最重单轴只有 5：只看单轴会误判限值 6 可通行，实际峰值 8 被挡住。
        seg6 = RoadSegment(7, "S", "E", 2, BridgeSpec(plateau, Fraction(6)))
        rev6 = review_bridge(seg6, layout_2axle(), (3, 5))
        self.assertFalse(rev6.feasible)
        self.assertEqual(rev6.excess, 2)

    def test_rational_peak_not_at_integer(self):
        # 分数折点曲线，峰值必须精确，不得退化成整数采样。
        line = InfluenceLine((
            (0, 0), (Fraction(1, 2), Fraction(1, 2)), (1, 0)
        ))
        seg = RoadSegment(3, "S", "E", 1, BridgeSpec(line, Fraction(10)))
        layout = AxleLayout("T", "R", (0,))
        rev = review_bridge(seg, layout, (3,))
        self.assertEqual(rev.peak, Fraction(3, 2))
        self.assertEqual(rev.witness_x, Fraction(1, 2))

    def test_different_segments_keep_own_curves(self):
        # 两条并行桥，曲线不同：同一车组一个可过、一个不可过。
        a = RoadSegment(1, "S", "E", 2, BridgeSpec(triangle_line(), Fraction(4)))
        plateau = InfluenceLine(((0, 0), (2, 1), (4, 1), (6, 0)))
        b = RoadSegment(2, "S", "E", 2, BridgeSpec(plateau, Fraction(4)))
        self.assertTrue(review_bridge(a, layout_2axle(), (3, 5)).feasible is False)
        self.assertTrue(review_bridge(b, layout_2axle(), (3, 5)).feasible is False)
        b_ok = RoadSegment(2, "S", "E", 2, BridgeSpec(plateau, Fraction(8)))
        self.assertTrue(review_bridge(b_ok, layout_2axle(), (3, 5)).feasible)

    def test_rejects_non_positive_loads(self):
        seg = RoadSegment(1, "S", "E", 2, BridgeSpec(triangle_line(), Fraction(4)))
        with self.assertRaises(RoutingError):
            review_bridge(seg, layout_2axle(), (0, 5))


class ShortestRouteTests(unittest.TestCase):
    def _archive(self, direct_limit: Fraction, direct_id=10, detour_id=20):
        return RouteArchive([
            RoadSegment(direct_id, "S", "E", 2,
                        BridgeSpec(triangle_line(), direct_limit)),
            RoadSegment(detour_id, "S", "E", 7, None),
        ])

    def test_textbook_route_choices(self):
        layout, loads = layout_2axle(), (3, 5)
        # 限值 4：直桥峰值 5 被挡，选耗时 7 的绕路。
        r4 = shortest_route(self._archive(Fraction(4)), "S", "E", layout, loads)
        self.assertEqual(r4.status, ROUTE_FEASIBLE)
        self.assertEqual((r4.time, r4.segment_path), (7, (20,)))
        # 限值 5：直桥放行，选耗时 2 的直路。
        r5 = shortest_route(self._archive(Fraction(5)), "S", "E", layout, loads)
        self.assertEqual((r5.time, r5.segment_path), (2, (10,)))
        # 同毛重 4/4：限值 4 即可走直路。
        r44 = shortest_route(self._archive(Fraction(4)), "S", "E", layout, (4, 4))
        self.assertEqual((r44.time, r44.segment_path), (2, (10,)))

    def test_tie_prefers_lexicographically_smaller_segment_sequence(self):
        # 两条并行无桥路同为耗时 7：编号小者胜。
        arch = RouteArchive([
            RoadSegment(20, "S", "E", 7, None),
            RoadSegment(10, "S", "E", 7, None),
        ])
        r = shortest_route(arch, "S", "E", layout_2axle(), (3, 5))
        self.assertEqual(r.segment_path, (10,))

    def test_lex_tie_through_different_node_paths(self):
        # S->A->E 与 S->B->E 同耗时；首段编号 1 < 2，走 A 线。
        arch = RouteArchive([
            RoadSegment(1, "S", "A", 2, None),
            RoadSegment(2, "S", "B", 2, None),
            RoadSegment(3, "A", "E", 3, None),
            RoadSegment(4, "B", "E", 3, None),
        ])
        r = shortest_route(arch, "S", "E", layout_2axle(), (1, 1))
        self.assertEqual(r.node_path, ("S", "A", "E"))
        self.assertEqual(r.segment_path, (1, 3))

    def test_blocked_route_gives_evidence(self):
        arch = RouteArchive([
            RoadSegment(1, "S", "M", 1, None),
            RoadSegment(2, "M", "X", 1, BridgeSpec(triangle_line(), Fraction(1))),
        ])
        r = shortest_route(arch, "S", "X", layout_2axle(), (3, 5))
        self.assertEqual(r.status, ROUTE_BLOCKED)
        self.assertEqual(r.reachable_nodes, ("M", "S", "X"))
        self.assertEqual(len(r.blocking), 1)
        evidence = r.blocking[0]
        self.assertEqual(evidence.segment_id, 2)
        self.assertEqual(str(evidence.review.peak), "5")
        self.assertEqual(str(evidence.review.excess), "4")
        self.assertEqual(evidence.review.witness_x, 4)

    def test_unreachable_target_outside_graph_reach(self):
        arch = RouteArchive([
            RoadSegment(1, "S", "M", 1, None),
            RoadSegment(2, "M", "X", 1, BridgeSpec(triangle_line(), Fraction(1))),
            RoadSegment(3, "Q", "R", 1, None),  # 与起点不连通
        ])
        r = shortest_route(arch, "S", "R", layout_2axle(), (3, 5))
        self.assertEqual(r.status, ROUTE_UNREACHABLE)
        self.assertEqual(r.reachable_nodes, ("M", "S", "X"))
        # 仍给出可达区域内挡住出口的桥证据。
        self.assertEqual([b.segment_id for b in r.blocking], [2])

    def test_unknown_endpoint_rejected(self):
        arch = self._archive(Fraction(4))
        with self.assertRaises(RoutingError):
            shortest_route(arch, "S", "Z", layout_2axle(), (3, 5))

    def test_invalid_reconstruction_refused(self):
        c = Catalog()
        c.define_mass("tractor", "T1", 1, 5)
        c.define_mass("trailer", "R1", 1, 5)
        c.apply_attach("T1", "R1")
        w = Warehouse(c)
        w.add_ticket(dict(ticket_id="WB", tractor="T1", catalog_seq=1,
                          ts=1, direction="IN", gross=8))
        reg = AxleRegistry()
        reg.register(layout_2axle())
        bad = reconstruct_record(
            PlatformRecord("WB", 2, ((0, 3), (4, 0))), w, reg  # 漏点
        )
        self.assertFalse(bad.valid)
        with self.assertRaises(AxleError):
            shortest_route_for_record(self._archive(Fraction(4)), "S", "E", bad)

    def test_bound_record_end_to_end(self):
        c = Catalog()
        c.define_mass("tractor", "T1", 1, 5)
        c.define_mass("trailer", "R1", 1, 5)
        c.apply_attach("T1", "R1")
        w = Warehouse(c)
        w.add_ticket(dict(ticket_id="WB-35", tractor="T1", catalog_seq=1,
                          ts=1, direction="IN", gross=8))
        reg = AxleRegistry()
        reg.register(layout_2axle())
        rec = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 0))), w, reg
        )
        r = shortest_route_for_record(self._archive(Fraction(4)), "S", "E", rec)
        self.assertEqual(r.segment_path, (20,))

    def test_archive_json_roundtrip_keeps_exact_rationals(self):
        arch = self._archive(Fraction(4, 3))
        back = RouteArchive.from_json(arch.to_json())
        self.assertEqual(back.segment(10).bridge.limit, Fraction(4, 3))
        self.assertEqual(
            back.segment(10).bridge.influence_line.points,
            triangle_line().points,
        )


class BruteForceRouteTests(unittest.TestCase):
    """随机小图：Dijkstra 标签结果与枚举全部简单路径的暴力结论逐项对照。"""

    @staticmethod
    def _all_simple_paths(arch, source, target, layout, loads):
        adj = {n: arch.outgoing(n) for n in arch.nodes()}

        def feasible(seg):
            if not seg.is_bridge:
                return True
            return review_bridge(seg, layout, loads).feasible

        results: list[tuple[int, tuple[int, ...]]] = []

        def walk(node, visited_edges, time, path):
            if node == target:
                results.append((time, tuple(path)))
                return
            for seg in adj.get(node, ()):
                if seg.segment_id in visited_edges or not feasible(seg):
                    continue
                walk(seg.target, visited_edges | {seg.segment_id},
                     time + seg.travel_time, path + [seg.segment_id])

        walk(source, set(), 0, [])
        if not results:
            return None
        return min(results)

    def test_random_graphs_match_brute_force(self):
        rng = random.Random(20261120)
        layout = AxleLayout("T", "R", (0, 2, 5))
        loads = (3, 4, 5)
        line = InfluenceLine(((0, 0), (2, 1), (5, Fraction(1, 2)), (8, 0)))
        for trial in range(60):
            nodes = ["S", "A", "B", "E"]
            segs = []
            sid = 0
            for u, v in itertools.permutations(nodes, 2):
                if rng.random() < 0.45:
                    sid += 1
                    if rng.random() < 0.5:
                        limit = Fraction(rng.randint(1, 12))
                        seg = RoadSegment(sid, u, v, rng.randint(1, 6),
                                          BridgeSpec(line, limit))
                    else:
                        seg = RoadSegment(sid, u, v, rng.randint(1, 6), None)
                    segs.append(seg)
            arch = RouteArchive(segs)
            if "S" not in arch.nodes() or "E" not in arch.nodes():
                continue  # 随机图可能完全没碰到起终点
            got = shortest_route(arch, "S", "E", layout, loads)
            brute = self._all_simple_paths(arch, "S", "E", layout, loads)
            if brute is None:
                self.assertNotEqual(got.status, ROUTE_FEASIBLE)
                # 底层可达性独立核对。
                seen = {"S"}
                stack = ["S"]
                while stack:
                    n = stack.pop()
                    for s in arch.outgoing(n):
                        if s.target not in seen:
                            seen.add(s.target)
                            stack.append(s.target)
                self.assertEqual(set(got.reachable_nodes), seen)
                if "E" in seen:
                    self.assertEqual(got.status, ROUTE_BLOCKED)
                else:
                    self.assertEqual(got.status, ROUTE_UNREACHABLE)
            else:
                self.assertEqual(got.status, ROUTE_FEASIBLE)
                self.assertEqual((got.time, got.segment_path), brute)


if __name__ == "__main__":
    unittest.main()
