"""短秤台轴重还原的端到端测试。

运行::

    python3 -m unittest tests.test_axle -v
"""

from __future__ import annotations

import itertools
import json
import unittest

from weighbridge import (
    AxleError,
    AxleLayout,
    AxleRegistry,
    BAD_READING,
    Catalog,
    DUPLICATE_POINT,
    END_NOT_ZERO,
    GROSS_MISMATCH,
    HISTORY_LAYOUT_MISSING,
    INCONSISTENT_READINGS,
    IN,
    MAX_AXLES,
    MISSING_POINT,
    NON_POSITIVE_AXLE,
    OUT,
    PlatformRecord,
    TICKET_NOT_FOUND,
    UNEXPECTED_POINT,
    Warehouse,
    expected_event_positions,
    expected_events,
    reconstruct_record,
    simulate_readings,
)


def build_bound_world():
    """目录 + 仓库 + 布局登记；两台历史车组，若干张票。"""
    c = Catalog()
    c.define_mass("tractor", "T1", 1, 5)
    c.define_mass("tractor", "T2", 1, 5)
    c.define_mass("trailer", "R1", 1, 5)
    c.define_mass("trailer", "R2", 1, 5)
    c.apply_attach("T1", "R1")  # seq 1: ('T1','R1')
    c.apply_detach("T1", "R1")  # seq 2: ('T1', None)
    c.apply_attach("T1", "R2")  # seq 3: ('T1','R2')
    c.apply_attach("T2", "R1")  # seq 4: ('T2','R1')

    w = Warehouse(c)
    tickets = [
        dict(ticket_id="WB-35", tractor="T1", catalog_seq=1, ts=10,
             direction=IN, gross=8),
        dict(ticket_id="WB-44", tractor="T1", catalog_seq=1, ts=11,
             direction=OUT, gross=8),
        dict(ticket_id="WB-OTHER", tractor="T1", catalog_seq=3, ts=12,
             direction=IN, gross=8),
        dict(ticket_id="WB-T2", tractor="T2", catalog_seq=4, ts=13,
             direction=IN, gross=8),
        dict(ticket_id="WB-SOLO", tractor="T1", catalog_seq=2, ts=14,
             direction=IN, gross=7),
    ]
    for t in tickets:
        w.add_ticket(t)

    reg = AxleRegistry()
    layout_35 = AxleLayout("T1", "R1", (0, 2))
    layout_other = AxleLayout("T1", "R2", (0, 3, 5))
    layout_t2 = AxleLayout("T2", "R1", (0, 2))
    layout_solo = AxleLayout("T1", None, (0,))
    for lay in (layout_35, layout_other, layout_t2, layout_solo):
        reg.register(lay)
    return c, w, reg, layout_35, layout_other, layout_t2, layout_solo


class LayoutAndRegistryTests(unittest.TestCase):
    def test_layout_validation(self):
        AxleLayout("T", "R", (0, 1, 3))
        with self.assertRaises(AxleError):
            AxleLayout("T", "R", (1, 2))           # 首轴不为 0
        with self.assertRaises(AxleError):
            AxleLayout("T", "R", (0, 0))           # 不严格递增
        with self.assertRaises(AxleError):
            AxleLayout("T", "R", (0, -1))          # 负位置
        with self.assertRaises(AxleError):
            AxleLayout("T", "R", ())               # 无轴
        with self.assertRaises(AxleError):
            AxleLayout("T", "R", tuple(range(MAX_AXLES + 1)))

    def test_registry_conflict_and_identity_lookup(self):
        reg = AxleRegistry()
        reg.register(AxleLayout("T1", "R1", (0, 2)))
        reg.register(AxleLayout("T1", "R1", (0, 2)))  # 同布局幂等
        with self.assertRaises(AxleError):
            reg.register(AxleLayout("T1", "R1", (0, 4)))
        with self.assertRaises(AxleError):
            reg.layout_for_identity(("T9", None))

    def test_registry_json_roundtrip(self):
        reg = AxleRegistry()
        reg.register(AxleLayout("T1", "R1", (0, 2, 5)))
        reg.register(AxleLayout("T1", None, (0,)))
        back = AxleRegistry.from_json(reg.to_json())
        self.assertEqual(back.identities(), reg.identities())
        self.assertEqual(
            back.layout_for_identity(("T1", "R1")).positions, (0, 2, 5)
        )


class ReconstructionTests(unittest.TestCase):
    def test_textbook_example_3_5(self):
        _, w, reg, layout, *_ = build_bound_world()
        rec = PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 0)))
        r = reconstruct_record(rec, w, reg)
        self.assertTrue(r.valid)
        self.assertEqual(r.axle_loads, (3, 5))
        self.assertEqual(r.identity, ("T1", "R1"))
        self.assertEqual(r.gross, 8)
        # 事件审计：x=2 同时是 1 轴出、2 轴进，动作后只剩后轴 5。
        by_pos = {e.position: e for e in r.events}
        self.assertEqual(by_pos[0].entering, (0,))
        self.assertEqual(by_pos[0].on_platform, (0,))
        self.assertEqual(by_pos[2].exiting, (0,))
        self.assertEqual(by_pos[2].entering, (1,))
        self.assertEqual(by_pos[2].on_platform, (1,))
        self.assertEqual(by_pos[2].computed, 5)
        self.assertEqual(by_pos[4].on_platform, ())
        self.assertEqual(by_pos[4].computed, 0)

    def test_same_gross_different_split_4_4(self):
        # 轴距 == 台长，半开区间下两轴永不同台，x=2 读数应为 4。
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-44", 2, ((0, 4), (2, 4), (4, 0))), w, reg
        )
        self.assertTrue(r.valid)
        self.assertEqual(r.axle_loads, (4, 4))

    def test_both_axles_on_platform_longer_deck(self):
        # 台长 4 > 轴距 2：x=2 与 x=3 两轴同台，读数 8。
        _, w, reg, layout, *_ = build_bound_world()
        readings = simulate_readings(layout, (3, 5), 4)
        self.assertEqual(readings, [(0, 3), (2, 8), (4, 5), (6, 0)])
        # 票面毛重需 8；复用 WB-35。
        r = reconstruct_record(PlatformRecord("WB-35", 4, readings), w, reg)
        self.assertTrue(r.valid, [i.code for i in r.issues])
        self.assertEqual(r.axle_loads, (3, 5))
        middle = next(e for e in r.events if e.position == 2)
        self.assertEqual(middle.on_platform, (0, 1))
        self.assertEqual(middle.computed, 8)

    def test_three_axle_same_position_entry_and_exit(self):
        _, w, reg, _, layout_other, _, _ = build_bound_world()
        # d=(0,3,5), L=3：进轴 0,3,5；出轴 3,6,8。x=3 同时进 1 轴、出 0 轴。
        events = expected_event_positions(layout_other, 3)
        self.assertEqual(events, [0, 3, 5, 6, 8])
        ev3 = next(e for e in expected_events(layout_other, 3) if e.position == 3)
        self.assertEqual(ev3.entering, (1,))
        self.assertEqual(ev3.exiting, (0,))
        self.assertEqual(ev3.on_platform, (1,))
        readings = simulate_readings(layout_other, (2, 3, 3), 3)
        r = reconstruct_record(PlatformRecord("WB-OTHER", 3, readings), w, reg)
        self.assertTrue(r.valid, [i.code for i in r.issues])
        self.assertEqual(r.axle_loads, (2, 3, 3))

    def test_solo_axle_vehicle(self):
        _, w, reg, _, _, _, layout_solo = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-SOLO", 5, ((0, 7), (5, 0))), w, reg
        )
        self.assertTrue(r.valid)
        self.assertEqual(r.axle_loads, (7,))

    def test_missing_point_is_located(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 3), (4, 0))), w, reg  # 漏 x=2
        )
        self.assertFalse(r.valid)
        self.assertEqual([i.code for i in r.issues], [MISSING_POINT])
        self.assertEqual(r.issues[0].location, "x=2")
        self.assertIsNone(r.axle_loads)
        with self.assertRaises(AxleError):
            r.require_valid()

    def test_unexpected_point_is_located(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 3), (1, 3), (2, 5), (4, 0))), w, reg
        )
        self.assertFalse(r.valid)
        self.assertEqual([i.code for i in r.issues], [UNEXPECTED_POINT])
        self.assertEqual(r.issues[0].location, "x=1")

    def test_duplicate_point(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord(
                "WB-35", 2, ((0, 3), (0, 3), (2, 5), (4, 0))
            ), w, reg
        )
        self.assertFalse(r.valid)
        self.assertEqual([i.code for i in r.issues], [DUPLICATE_POINT])
        self.assertEqual(r.issues[0].location, "x=0")

    def test_negative_reading(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, -1), (2, 5), (4, 0))), w, reg
        )
        self.assertFalse(r.valid)
        self.assertIn(BAD_READING, [i.code for i in r.issues])

    def test_end_not_zero(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 2))), w, reg
        )
        self.assertFalse(r.valid)
        codes = [i.code for i in r.issues]
        self.assertIn(END_NOT_ZERO, codes)
        # 末端非零同时使方程组与 (3,5) 解矛盾（末行 0·x = 2）。
        self.assertIn(INCONSISTENT_READINGS, codes)

    def test_inconsistent_readings_located_to_conflicting_positions(self):
        _, w, reg, *_ = build_bound_world()
        # x=2 与 x=4 台面都已空（0/1 系数行全零），却读到 5 和 2：直接矛盾；
        # 末端读数 2 同时触发 END_NOT_ZERO，两个问题都应可定位。
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 2))), w, reg
        )
        self.assertFalse(r.valid)
        codes = [i.code for i in r.issues]
        self.assertIn(INCONSISTENT_READINGS, codes)
        issue = next(i for i in r.issues if i.code == INCONSISTENT_READINGS)
        # x=4 是全零系数行却读到 2，消元首先在此定位出矛盾。
        self.assertIn("x=4", issue.location)
        self.assertIsNone(r.axle_loads)

    def test_non_positive_axle_rejected(self):
        _, w, reg, *_ = build_bound_world()
        # 进轴读数 0 解出首轴重 0；末端也必须为 0，故毛重核对另报。
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 0), (2, 5), (4, 0))), w, reg
        )
        self.assertFalse(r.valid)
        codes = [i.code for i in r.issues]
        self.assertIn(NON_POSITIVE_AXLE, codes)
        self.assertIn(GROSS_MISMATCH, codes)

    def test_gross_mismatch(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("WB-35", 2, ((0, 4), (2, 5), (4, 0))), w, reg
        )  # 轴重和 9 != 毛重 8
        self.assertFalse(r.valid)
        self.assertEqual([i.code for i in r.issues], [GROSS_MISMATCH])
        self.assertEqual(r.axle_loads, (4, 5))  # 解仍给出，只是不可用

    def test_unknown_ticket(self):
        _, w, reg, *_ = build_bound_world()
        r = reconstruct_record(
            PlatformRecord("GHOST", 2, ((0, 1), (2, 0))), w, reg
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.issues[0].code, TICKET_NOT_FOUND)

    def test_history_layout_missing_blocks_foreign_group(self):
        # T2/R1 与 T1/R1 轴距即便相同，也是另一历史车组；这里故意不登记 T2。
        c, w, _, *_ = build_bound_world()
        reg = AxleRegistry()
        reg.register(AxleLayout("T1", "R1", (0, 2)))
        r = reconstruct_record(
            PlatformRecord("WB-T2", 2, ((0, 3), (2, 5), (4, 0))), w, reg
        )
        self.assertFalse(r.valid)
        self.assertEqual(r.issues[0].code, HISTORY_LAYOUT_MISSING)
        self.assertEqual(r.identity, ("T2", "R1"))

    def test_same_gross_ticket_cannot_substitute_layout(self):
        # 两张毛重都是 8 的票属于不同车组（不同轴距），布局按身份各取各的。
        _, w, reg, _, layout_other, layout_t2, _ = build_bound_world()
        r_t2 = reconstruct_record(
            PlatformRecord(
                "WB-T2", 2, ((0, 3), (2, 5), (4, 0))
            ), w, reg
        )
        self.assertTrue(r_t2.valid)
        # 把 T2 的记录错绑到 T1/R2（三轴车）的票上：点位集合直接不匹配。
        r_wrong = reconstruct_record(
            PlatformRecord(
                "WB-OTHER", 2, ((0, 3), (2, 5), (4, 0))
            ), w, reg
        )
        self.assertFalse(r_wrong.valid)
        # 三轴距布局期望 0,2,3,5,7 五个事件点，两轴记录既漏点又多点。
        codes = {i.code for i in r_wrong.issues}
        self.assertIn(MISSING_POINT, codes)
        self.assertIn(UNEXPECTED_POINT, codes)


class SimulationConsistencyTests(unittest.TestCase):
    def test_simulated_readings_always_reconstruct(self):
        # 暴力：小轴距集合 + 台长 + 正整数轴重，合成记录必须精确还原自身。
        rng = range(1, 4)
        for span in rng:
            positions = (0,) + tuple(2 * i + 1 for i in range(1, span + 1))
            layout = AxleLayout("T", "R", positions)
            for L in range(1, 9):
                for loads in itertools.product(range(1, 4), repeat=len(positions)):
                    readings = simulate_readings(layout, loads, L)
                    rec = PlatformRecord(
                        "T", L, readings
                    )
                    events = expected_events(layout, L)
                    self.assertEqual(
                        [p for p, _ in readings],
                        [e.position for e in events],
                    )
                    # 直接对核心求解做断言（绕开仓库绑定）。
                    from weighbridge.axle import _reconstruct_core
                    gross = sum(loads)
                    r = _reconstruct_core(rec, layout, gross)
                    self.assertTrue(r.valid, (positions, L, loads, r.issues))
                    self.assertEqual(r.axle_loads, loads)
                    self.assertEqual(r.events[-1].computed, 0)


class SerializationTests(unittest.TestCase):
    def test_record_and_result_roundtrip(self):
        _, w, reg, *_ = build_bound_world()
        rec = PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 0)))
        back = PlatformRecord.from_dict(rec.to_dict())
        self.assertEqual(back, rec)
        r = reconstruct_record(back, w, reg)
        data = r.to_dict()
        self.assertEqual(json.loads(json.dumps(data))["axle_loads"], [3, 5])
        self.assertEqual(data["events"][1]["on_platform"], [1])


if __name__ == "__main__":
    unittest.main()
