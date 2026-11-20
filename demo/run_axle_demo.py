"""旧车次短秤台记录还原 + 历史线路桥面荷载复核的可复跑演示。

运行::

    python3 -m demo.run_axle_demo          # 打印复核结论
    python3 -m demo.run_axle_demo --json   # 另写 demo/out/axle_review_*.json

与 :mod:`demo.run_demo` 完全独立，不改动原有单票/双票净装载演示及其输出。

覆盖题面要求
------------
1. 题面算例实跑：轴位 0/2、台长 2、读数 3/5/0、毛重 8 -> 轴重 (3, 5)。
2. 两辆总重相同（毛重都为 8）的车：3/5 与 4/4，三角影响线限值 4 时一挡一放；
   限值 5 时 3/5 也放行；直路耗时 2、无桥绕路 7 时的选路变化。
3. 两轴同时对桥面峰值有贡献的记录（平台曲线，峰值 8 = 3+5，最重单轴仅 5）。
4. 漏点、读数矛盾、末端不归零、历史车组不匹配、同毛重别张票不可代替的定位。
5. 无路可选：底层可达范围 + 挡住出口的桥段证据；以及底层不可达。
6. 外部组合快照路径：``Warehouse(None)`` 仅靠快照完成绑定与还原。
"""

from __future__ import annotations

import argparse
import json
import pathlib

from fractions import Fraction

from weighbridge import (
    AxleLayout,
    AxleRegistry,
    Catalog,
    HISTORY_LAYOUT_MISSING,
    IN,
    InfluenceLine,
    OUT,
    PlatformRecord,
    ROUTE_BLOCKED,
    ROUTE_UNREACHABLE,
    BridgeSpec,
    RoadSegment,
    RouteArchive,
    Warehouse,
    reconstruct_record,
    review_bridge,
    shortest_route_for_record,
    simulate_readings,
)


# ---------------------------------------------------------------- 合成世界
def build_world():
    """目录/票/轴布局：三台历史车组，全部本地合成整数记录。"""
    c = Catalog()
    c.define_mass("tractor", "T-35", 1, 5)
    c.define_mass("tractor", "T-44", 1, 5)
    c.define_mass("tractor", "T-99", 1, 5)
    c.define_mass("trailer", "R-A", 1, 5)
    c.define_mass("trailer", "R-B", 1, 5)
    c.define_mass("trailer", "R-C", 1, 5)
    c.apply_attach("T-35", "R-A")   # seq 1: ('T-35','R-A') 两轴
    c.apply_attach("T-44", "R-B")   # seq 2: ('T-44','R-B') 两轴
    c.apply_attach("T-99", "R-C")   # seq 3: ('T-99','R-C') 三轴，轴距不同

    w = Warehouse(c)
    # 两张票毛重完全相同（8），但属于不同历史车组。
    w.add_ticket(dict(ticket_id="WB-35", tractor="T-35", catalog_seq=1,
                      ts=100, direction=IN, gross=8))
    w.add_ticket(dict(ticket_id="WB-44", tractor="T-44", catalog_seq=2,
                      ts=200, direction=OUT, gross=8))
    # 同车组、同毛重的另一张票：不能代替 WB-35 去复核它的记录。
    w.add_ticket(dict(ticket_id="WB-35B", tractor="T-35", catalog_seq=1,
                      ts=101, direction=OUT, gross=8))
    # 三轴车组同样毛重 8（2+3+3）；同毛重票不可跨车组代替。
    w.add_ticket(dict(ticket_id="WB-99", tractor="T-99", catalog_seq=3,
                      ts=250, direction=IN, gross=8))
    # 供无效记录演示的票。
    w.add_ticket(dict(ticket_id="WB-BAD", tractor="T-35", catalog_seq=1,
                      ts=300, direction=IN, gross=8))

    reg = AxleRegistry()
    layout_35 = AxleLayout("T-35", "R-A", (0, 2))
    layout_44 = AxleLayout("T-44", "R-B", (0, 2))
    layout_99 = AxleLayout("T-99", "R-C", (0, 3, 5))
    reg.register(layout_35)
    reg.register(layout_44)
    reg.register(layout_99)
    return c, w, reg, layout_35, layout_44, layout_99


def build_archives():
    """题面线路：直路桥耗时 2，无桥绕路耗时 7；另建无路档案。"""
    triangle = InfluenceLine(((0, 0), (2, 1), (4, 0)))
    plateau = InfluenceLine(((0, 0), (2, 1), (4, 1), (6, 0)))

    def main_archive(limit):
        return RouteArchive([
            RoadSegment(10, "S", "E", 2, BridgeSpec(triangle, Fraction(limit))),
            RoadSegment(20, "S", "E", 7, None),
        ])

    plateau_bridge = RouteArchive([
        RoadSegment(30, "S", "E", 2, BridgeSpec(plateau, Fraction(6))),
        RoadSegment(40, "S", "E", 9, None),
    ])
    no_route = RouteArchive([
        RoadSegment(1, "S", "M", 1, None),
        RoadSegment(2, "M", "X", 1, BridgeSpec(triangle, Fraction(1))),
        RoadSegment(3, "Q", "R", 1, None),  # 与 S 不连通的孤岛
    ])
    return main_archive, plateau_bridge, no_route, triangle, plateau


# ---------------------------------------------------------------- 报告
def report(c, w, reg, layout_35, layout_44, layout_99):
    out: dict = {"reconstruction": {}, "bridge_review": {}, "routing": {},
                 "invalid_records": {}, "snapshot_path": {}, "checks": {}}

    # 1) 题面算例：3/5 还原
    rec35 = PlatformRecord("WB-35", 2, ((0, 3), (2, 5), (4, 0)))
    r35 = reconstruct_record(rec35, w, reg)
    assert r35.valid
    out["reconstruction"]["textbook_3_5"] = r35.to_dict()

    # 2) 同毛重不同分配：4/4（台长=轴距，半开区间下两轴从不同台）
    rec44 = PlatformRecord("WB-44", 2, ((0, 4), (2, 4), (4, 0)))
    r44 = reconstruct_record(rec44, w, reg)
    assert r44.valid
    out["reconstruction"]["same_gross_4_4"] = r44.to_dict()

    # 3) 两轴同台记录：台长 4，x=2/3 两轴同在，读数 8
    joint_readings = simulate_readings(layout_35, (3, 5), 4)
    r_joint = reconstruct_record(PlatformRecord("WB-35B", 4, joint_readings), w, reg)
    assert r_joint.valid
    out["reconstruction"]["both_axles_on_platform"] = {
        "readings": joint_readings, "result": r_joint.to_dict(),
    }

    main_archive, plateau_archive, no_route, triangle, plateau = build_archives()

    # 桥面峰值的精确事实
    seg_tri = RoadSegment(99, "S", "E", 2, BridgeSpec(triangle, Fraction(4)))
    seg_plat = RoadSegment(98, "S", "E", 2, BridgeSpec(plateau, Fraction(6)))
    rev35 = review_bridge(seg_tri, layout_35, (3, 5))
    rev44 = review_bridge(seg_tri, layout_44, (4, 4))
    rev_plat = review_bridge(seg_plat, layout_35, (3, 5))
    out["bridge_review"] = {
        "triangle_3_5": rev35.to_dict(),
        "triangle_4_4": rev44.to_dict(),
        "plateau_joint_contribution": rev_plat.to_dict(),
    }

    # 4) 选路变化：限值 4 / 限值 5；3/5 与 4/4
    out["routing"]["limit4_3_5"] = shortest_route_for_record(
        main_archive(4), "S", "E", r35
    ).to_dict()
    out["routing"]["limit5_3_5"] = shortest_route_for_record(
        main_archive(5), "S", "E", r35
    ).to_dict()
    out["routing"]["limit4_4_4"] = shortest_route_for_record(
        main_archive(4), "S", "E", r44
    ).to_dict()
    # 平台曲线：只看最重单轴(5)会误判限值6可行，实际峰值8被挡，绕路。
    out["routing"]["plateau_limit6"] = shortest_route_for_record(
        plateau_archive, "S", "E", r35
    ).to_dict()

    # 5) 无路可选：BLOCKED 与 UNREACHABLE 的证据
    blocked = shortest_route_for_record(no_route, "S", "X", r35)
    unreachable = shortest_route_for_record(no_route, "S", "R", r35)
    assert blocked.status == ROUTE_BLOCKED
    assert unreachable.status == ROUTE_UNREACHABLE
    out["routing"]["blocked"] = blocked.to_dict()
    out["routing"]["unreachable"] = unreachable.to_dict()

    # 6) 无效记录全部可定位，且不能进入线路复核
    invalid = {
        # 漏掉 x=2 的进/出轴读数
        "missing_point": PlatformRecord("WB-BAD", 2, ((0, 3), (4, 0))),
        # 真正的读数矛盾（台长 4，x=2..4 两轴同台）：x=2 要求 w1+w2=9、
        # x=4 要求 w2=5，推出 w1=4，与 x=0 的 w1=3 冲突；末端 x=6 仍归零。
        "contradiction": PlatformRecord(
            "WB-BAD", 4, ((0, 3), (2, 9), (4, 5), (6, 0))
        ),
        # 可解但轴重和 9 != 毛重 8
        "gross_mismatch": PlatformRecord("WB-BAD", 2, ((0, 4), (2, 5), (4, 0))),
        # 末端不归零，同时与零系数行矛盾
        "end_not_zero": PlatformRecord("WB-BAD", 2, ((0, 3), (2, 5), (4, 2))),
    }
    for name, rec in invalid.items():
        rr = reconstruct_record(rec, w, reg)
        assert not rr.valid
        out["invalid_records"][name] = rr.to_dict()

    # 历史车组不匹配（一）：登记册里没有该车组的布局，直接拒绝。
    reg_partial = AxleRegistry()
    reg_partial.register(layout_35)
    missing_layout = reconstruct_record(rec44, w, reg_partial)
    assert not missing_layout.valid
    assert missing_layout.issues[0].code == HISTORY_LAYOUT_MISSING
    out["invalid_records"]["history_layout_missing"] = missing_layout.to_dict()

    # 历史车组不匹配（二）：把两轴秤台记录错绑到同毛重 8 的三轴车组票上，
    # 期望事件点集合不同，漏点/多点同时暴露，记录不可用。
    mismatched = reconstruct_record(
        PlatformRecord("WB-99", 2, ((0, 3), (2, 5), (4, 0))), w, reg
    )
    assert not mismatched.valid
    out["invalid_records"]["history_group_mismatch"] = mismatched.to_dict()

    # 同毛重的正确三轴记录：2/3/3、台长 3，与两轴车组互不可代替。
    rec99 = PlatformRecord(
        "WB-99", 3, simulate_readings(layout_99, (2, 3, 3), 3)
    )
    r99 = reconstruct_record(rec99, w, reg)
    assert r99.valid and r99.axle_loads == (2, 3, 3)
    out["reconstruction"]["three_axle_same_gross_8"] = r99.to_dict()

    out["invalid_records"]["same_gross_other_ticket_note"] = (
        "WB-35 / WB-44 / WB-99 毛重同为 8，但复核只接受记录自身绑票还原出的 "
        "车组轴距与轴重：(3,5)@(0,2)、(4,4)@(0,2)、(2,3,3)@(0,3,5) 互不可代替。"
    )

    # 同毛重别张票不可代替：用 WB-35B 自己的记录走一次，身份仍是 T-35/R-A。
    assert r_joint.identity == ("T-35", "R-A")

    # 7) 外部快照路径：无 Catalog 的仓库，仅凭快照完成绑定与还原
    snap = c.snapshot_at(1, "T-35")
    w_ext = Warehouse(None)
    w_ext.register_snapshot(snap.to_json())
    w_ext.add_ticket(dict(ticket_id="WB-35", tractor="T-35", catalog_seq=1,
                          ts=100, direction=IN, gross=8))
    r_ext = reconstruct_record(rec35, w_ext, reg)
    assert r_ext.valid and r_ext.axle_loads == (3, 5)
    out["snapshot_path"] = {
        "snapshot": json.loads(snap.to_json()),
        "reconstruction": r_ext.to_dict(),
        "route_limit4": shortest_route_for_record(
            main_archive(4), "S", "E", r_ext
        ).to_dict(),
    }

    out["checks"] = {
        "warehouse_reconcile": w.reconcile(),
        "catalog_verify": c.verify(),
        "statuses": {
            "blocked": blocked.status, "unreachable": unreachable.status,
            "feasible_example": out["routing"]["limit5_3_5"]["status"],
        },
    }
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true", help="写出 demo/out/axle_review_*.json")
    args = ap.parse_args()

    c, w, reg, layout_35, layout_44, layout_99 = build_world()
    data = report(c, w, reg, layout_35, layout_44, layout_99)
    print(json.dumps(data, indent=2, ensure_ascii=False))

    if args.json:
        out = pathlib.Path(__file__).parent / "out"
        out.mkdir(exist_ok=True)
        (out / "axle_review_report.json").write_text(
            json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        main_archive, plateau_archive, no_route, _, _ = build_archives()
        (out / "axle_route_archive.json").write_text(
            main_archive(4).to_json(), encoding="utf-8"
        )
        (out / "axle_registry.json").write_text(reg.to_json(), encoding="utf-8")
        print(f"\nwrote {out}/axle_review_report.json", flush=True)


if __name__ == "__main__":
    main()
