# 挂车换接下的双向过磅与净装载事实仓

一台牵引车第一次带空挂车进厂、第二次却换了另一挂车出厂时，若按车牌配对两次
毛重，会把两套不同空车质量混在一起，制造不存在的净货重。本仓库把问题拆成
三个互相独立的模块：

* **车辆组合目录** (`weighbridge/catalog.py`)：只管车辆稳定身份、空车质量整数
  区间 `[lo, hi]`（kg）以及按严格递增序号发生的组合动作。它不知道磅单。
* **过磅事实仓** (`weighbridge/warehouse.py`)：消费目录在票面序号处的 *公开组合
  产品*，把毛重换算成货重区间、组织运输、计算净装卸量的精确区间、处理重评。
* **短秤台还原与线路复核** (`weighbridge/axle.py`、`weighbridge/routing.py`)：
  把短秤台记录绑定到 *具体磅单*，按票面历史车组的轴距布局精确还原每根轴的
  正整数载重，再用原车组轴距/轴重在有理数分段线性影响线上求桥面峰值，交付
  最短时间可行线路与无路时的堵点证据。

全部输入为本地合成整数记录，不连接地磅或车牌识别设备。Python 3.11，仅用标准
库（SQLite + `fractions.Fraction`）。

## 快速开始
```bash
python3 -m unittest discover -s tests -v     # 90 个测试
python3 -m demo.run_demo                     # 合成案例：净装载事实仓
python3 -m demo.run_demo --json              # 另写 demo/out/*.json
python3 -m demo.run_axle_demo                # 合成案例：轴重还原 + 线路复核
python3 -m demo.run_axle_demo --json         # 另写 demo/out/axle_review_*.json
```

```python
from weighbridge import Catalog, Warehouse, LOAD, IN, OUT

c = Catalog()
c.define_mass("tractor", "T1", 7000, 7200)   # 整数 kg，lo <= hi
c.define_mass("trailer", "R1", 1500, 1600)
c.define_mass("trailer", "R2", 2000, 2200)

c.apply_attach("T1", "R1")        # seq 1
c.apply_detach("T1", "R1")        # seq 2
c.apply_attach("T1", "R2")        # seq 3
print(c.product_at(1, "T1").stable_identity())   # ('T1', 'R1')，历史不被换挂覆盖

w = Warehouse(c)
w.add_ticket(dict(ticket_id="IN",  tractor="T1", catalog_seq=1, ts=100,
                  direction=IN,  gross=9000))
w.add_ticket(dict(ticket_id="OUT", tractor="T1", catalog_seq=3, ts=200,
                  direction=OUT, gross=12000))

ev = w.preview(LOAD, "IN", "OUT")  # DRAFT：只算，不占票
print(ev.net_lo, ev.net_hi)        # 2300 2600（整数空重区间上的精确极值）
w.confirm(ev.trip_id)              # CONFIRMED：此刻才占票
print(w.reconcile())               # [] 表示核对通过
```

## 目录语义

* **稳定身份**：实体 ID 为 `tractor:<id>` / `trailer:<id>`，永不复用。
* **动作序号**：`attach` / `detach` / `set_mass` 的序号严格递增；失败不消耗
  序号。**批动作** `apply_batch` 在单事务内执行，任一步失败整批回滚（序号、
  绑定、产品、批次记录全部不存在）。
* **组合约束**：一个挂车同时至多一台牵引车；一台牵引车同时至多一个挂车。
  由两枚 `WHERE detach_seq IS NULL` 的部分唯一索引强制。
* **组合产品**：每次 `attach`/`detach` 产生一个 `EMPTY` 产品；`set_mass` 产生
  `RELABEL` 产品。产品带 `relation_src`（`initial`/`attach`/`detach`/
  `mass_correction`）、`attach_seq`、`predecessor`。
* **历史不被覆盖**：产品的质量是落盘时的快照。查询任意序号：

  ```python
  c.product_at(seq, tractor)   # 该序号处已生效的 live 产品
  c.history("trailer", "R1")   # 该实体参与过的全部产品（含更正版）
  c.snapshot_at(seq, tractor)  # 可跨系统交换的 JSON 快照
  c.corrected_product(p)       # 同一挂接 + attach_seq 的最新更正版
  ```

  换挂之后，旧序号处的旧产品原样保留；`corrected_product` 只沿同一
  `(identity, attach_seq)` 的 `predecessor` 链走，绝不从 "当前挂接" 猜历史。

## 仓库语义

**单票货重**：`tare` 为组合产品各实体空车质量之和，
`goods = gross - tare`，可行区间
`[max(0, gross - tare_hi), gross - tare_lo]`；`gross < tare_lo`（上界为负）时该票
不合物理约束。`explain_ticket(tid)` 给出毛重、`tare`、`goods`、每个实体区间与
宽度——即 "这张票为什么是这个货重、哪些实体贡献了不确定性"。

**运输**：`preview(kind, ticket_in, ticket_out)` 显式选定一 IN 一 OUT 票，绝不按
时间就近猜配。IN 时间不得晚于 OUT；两票必须同车；一票不能同时占两个角色。
结果状态：

| 状态 | 含义 |
| --- | --- |
| `DRAFT` | 预览，未占票；失败时为不可确认的草稿 |
| `CONFIRMED` | 已确认；两票写进 `confirmed_ticket`，此后不可被别的运输占用 |
| `REVIEW` | 对已确认运输的重开/重评 |
| `SUPERSEDED` | 未确认的旧草稿/复核被同族新结果取代 |

**净装卸量**：

* 组合完全相同（同一牵引车 + 同一挂车）：空车质量是同一未知量，在毛重差中
  **精确抵消**，`net == |gross_out - gross_in|`，与区间宽度无关。
* 换挂：共享实体（如牵引车）系数为 0，精确抵消；不同实体以目录区间参与。
  在 "两张票货重非负 + 运输方向" 的联合可行域上，对净装卸量做**精确整数分支
  定界**（实数 LP 松弛用有理数顶点枚举），输出最小/最大净装卸量与货流区间。
  可行域为空则拒绝，区分 `DIRECTION_IMPOSSIBLE` 与 `TICKET_INFEASIBLE`。
* 绝不使用区间中点或实数顶点作为结算值。

`contributions` 逐个实体给出 `role`（`cancelled`/`fixed`/`uncertain`）、线性系数与
区间宽度。

**重评与重开**：

```python
c.apply_set_mass("trailer", "R1", 1500, 1600)  # 目录迟到更正
re = w.reopen(trip_id)        # 新增 REVIEW 运输，带 predecessor_trip
w.evaluation_history(re.trip_id)  # 旧（CONFIRMED）与新（REVIEW）都在，链式可追溯
```

* 已确认运输及其评估是历史事实，不会被覆盖或标为 `superseded`。
* 被确认运输占用的票不会被重新记成另一运输（重评沿用同一票与角色）。
* 重评默认对原评估中的 *历史产品* 取 `corrected_product`，而不是套用当前挂接。

**外部组合快照**：`Warehouse(None)` 可完全不依赖目录运行；用
`register_snapshot(...)` 导入与目录相同 JSON 格式的快照，按票面序号取产品。
若某张票的序号没有对应快照，直接报错——绝不回退到当前挂接去猜。

**核对**：`w.reconcile()`（`w.verify()` 同义，含目录自检）返回不一致列表，覆盖：
确认运输的两票方向/同车/时间序与可行性、确认占票唯一、每条评估在存下的快照
上可重算、同身份精确抵消、重评前序存在。

## 短秤台轴重还原（`weighbridge/axle.py`）

**绑定与布局**：`AxleRegistry` 按目录产品的 *稳定身份* `(tractor, trailer)`
登记 `AxleLayout`——相对前轴的整数轴距 `d`，首轴 0、向车尾严格递增，每车至多
16 轴。`reconstruct_record(record, warehouse, registry)` 先按 `ticket_id` 取磅单，
再按票面 `catalog_seq` 取历史组合产品（目录或外部快照，绝不从当前挂接回填），
最后按身份选当时布局。同毛重的别张票属于别的车组，不能代替。

**半开区间秤台模型**：台长 `L`，前轴位于 `x` 时轴 `i` 在台上当且仅当
`0 ≤ x − d_i < L`。进轴位置 `d_i`、出轴位置 `d_i + L`；一次进轴与一次出轴可
落在同一位置，设备记录全部同位动作完成后的台面总重，进入前为零。事件位置
并集由 `expected_events` 给出，`simulate_readings` 按同一模型合成合规记录。

**精确还原与核对**：在每个事件位置建立 0/1 在台方程组，高斯-若尔当消元全程
使用 `Fraction`（每行携带原始方程标签，矛盾时定位到具体读数位置），要求：

1. 读数位置与事件位置集合完全一致——漏点 `MISSING_POINT`、多点
   `UNEXPECTED_POINT`、重复点 `DUPLICATE_POINT`、负读数 `BAD_READING` 各自可定位；
2. 方程组相容（`INCONSISTENT_READINGS`，附矛盾涉及的位置）；
3. 每根轴载重为正整数（`NON_INTEGER_AXLE` / `NON_POSITIVE_AXLE`）；
4. 末轴出后台面归零（`END_NOT_ZERO`）；
5. 轴重和等于磅单毛重（`GROSS_MISMATCH`）；
6. 历史车组无布局时 `HISTORY_LAYOUT_MISSING`，磅单不存在时 `TICKET_NOT_FOUND`。

无效结果 `valid=False` 且带完整问题码与事件审计轨迹；`require_valid()` 会抛错，
线路复核入口因此不可能消费问题记录。

```python
rec = PlatformRecord("WB-35", platform_length=2,
                     readings=((0, 3), (2, 5), (4, 0)))
r = reconstruct_record(rec, warehouse, registry)   # 毛重 8 -> 轴重 (3, 5)
r.valid, r.axle_loads, [(e.position, e.computed) for e in r.events]
```

## 线路档案与桥面复核（`weighbridge/routing.py`）

**档案**：`RouteArchive` 是有向图，`RoadSegment` 带正整数行程时间；桥面路段
附 `BridgeSpec`（仅用于此档案的 `InfluenceLine` + 有理数 `limit`）。影响线折点
横坐标严格递增、区间两端取零、区间外为零，每条曲线至多 40 个折点；坐标与
取值接受 `int` / `Fraction` / `"p/q"` / `[p,q]`，拒绝浮点，JSON 可无损往返。

**精确峰值**：车组反应 `R(x) = Σ_i w_i·f(x−d_i)` 是若干分段仿射函数之和，
最大值只可能出现在折点平移并集上，故在有限候选点上用 `Fraction` 求值即得
*整个连续过桥过程* 的精确最大值，并给一个见证位置与该位置的逐轴贡献。
`peak ≤ limit` 放行，**等于上限即可通行**。不同路段持有不同曲线；峰值是全轴
加权和，不能退化成只看最重单轴。

**最短可行线路**：`shortest_route(...)` / `shortest_route_for_record(...)` 在
"桥面可通行" 子图上跑带字典序标签的 Dijkstra，标签为
`(累计时间, 路段编号元组, 节点路径元组)`：时间最短优先，时间相同取路段编号
序列字典序最小。后者只接受绑票且核对通过的 `Reconstruction`。

**无路可选**：返回 `BLOCKED`（终点在底层可达范围内）或 `UNREACHABLE`（不在），
并给出从起点在底层有向图上可达的节点/路段集合，以及从可达区域出发、因超限
不可通行的每一桥段的证据：精确峰值、上限、超限量、见证位置、逐轴贡献。

```python
triangle = InfluenceLine(((0, 0), (2, 1), (4, 0)))
archive = RouteArchive([
    RoadSegment(10, "S", "E", 2, BridgeSpec(triangle, Fraction(4))),  # 直路桥
    RoadSegment(20, "S", "E", 7, None),                                # 无桥绕路
])
shortest_route_for_record(archive, "S", "E", reconstruction_3_5)  # 限值4 -> 绕路(7)
# 限值 5 -> 直路(2)；同毛重 4/4 车组在限值 4 时已可走直路
```

## 容量

目录 100 实体（`define_mass` 时强制）；仓库 1000 张票、200 次运输。同一车同一秒
可有多张不同 `ticket_id` 的票。车组至多 16 轴（`MAX_AXLES`），每条影响线至多
40 个折点（`MAX_BREAKPOINTS`）。

## 文件

```
weighbridge/catalog.py      独立车辆组合目录
weighbridge/warehouse.py    过磅事实仓 + 精确整数求解
weighbridge/axle.py         短秤台记录绑票还原 + 问题定位
weighbridge/routing.py      影响线峰值 + 最短可行线路 + 堵点证据
weighbridge/__init__.py     公共 API
tests/test_weighbridge.py   46 个测试（含小整数区间暴力核对极值）
tests/test_axle.py          21 个测试（含合成记录暴力自洽）
tests/test_routing.py       23 个测试（含 60 张随机图暴力路径对照）
demo/run_demo.py            净装载合成案例（原演示，未改动）
demo/run_axle_demo.py       轴重还原 + 线路复核合成案例
demo/out/                   演示输出（--json 时生成）
```

## 覆盖的案例

* 同车同挂精确抵消（含极宽空车区间）
* 仅换挂：共享牵引车抵消、两挂车参与整数区间极值（与暴力枚举逐个核对）
* 目录迟到更正：已确认运输不变，新增带前序身份的重评
* 方向不可能、单票物理不可行、两票区间不相交
* 同票竞争：已确认运输占票后另一运输确认失败，且失败不改变状态/不占票
* 重开：多轮更正形成可追溯评估链
* 外部快照独立运行、快照冲突拒绝、缺失历史产品拒绝
* 批动作原子回滚、序号严格递增、单挂车/单牵引车约束、容量上限

短秤台与线路复核覆盖：

* 题面算例实跑：轴位 0/2、台长 2、读数 3/5/0、毛重 8 还原为轴重 (3,5)
* 同位置进/出轴事件、半开区间右端出轴、台长大于轴距时多轴同台
* 同毛重不同轴重分配 (3,5) 与 (4,4)：三角曲线限值 4 一挡一放，限值 5 均放行；
  直路耗时 2、绕路 7 的选路切换
* 平台影响线下两轴同时贡献峰值（8 = 3+5，最重单轴仅 5，限值 6 被正确挡住）
* 分数折点的精确峰值、等于上限放行、不同路段各自曲线
* 漏点/多点/重复点/负读数/矛盾读数/末端不归零/非正轴重/毛重不符全部带位置定位
* 历史车组缺布局、跨车组错绑（事件点集合不匹配）、同毛重票不可代替
* 最短时间相同的字典序兜底（含不同节点路径）、BLOCKED/UNREACHABLE 证据
* 外部快照仓库（`Warehouse(None)`）独立完成绑票还原与选路
* 随机有向图上 Dijkstra 结果与全简单路径枚举逐项对照（60 图）
