# 天机遥操作：回归定位、官方实现对照与诊断

> 本文记录收集完整实机快照前的排查和撤回步骤。后续已根据七条实测故障完成核心控制修改，当前实现与验证见[修复说明](teleop_servo_20260915.md)。下文“当前”指该历史排查阶段。

## 当前结论（2026-09-15）

上轮新增的 `tj_ik_continuous` 和单帧 `step bounds` 已撤回。它们把几何可达的目标变成频繁硬暂停；原生桥接和执行器恢复 SDK `NEAR_REF` 选解。**撤回只消除这次新增的拒绝条件，不能宣称原始内收和 J5/J6 限位已解决。**

原始日志包含 20 次左臂 IK 限位（17 次 J6 约 −60°，3 次 J5 约 +170°），但没有失败时的完整目标、参考关节或实际关节。它证明当时选出的解越限，不能证明该末端目标没有其他合法解，也不能确定观察到的内收属于哪条几何支路。

本轮新增失败时的 `[IK诊断]` 数值快照和纯离线分析工具。没有把官方固定肘平面、自动重试、关节裁剪或新优化器接入运动链。

## 1. 新增卡死的具体原因

执行器上轮的时间预算为：

```text
dt = min(5 ms, 本次 IK 开始时间 − 上次 driver.submit 返回时间)
max_step = 最大关节速度 × 配置比例 × dt
```

但末端插值目标按本次采样时刻推进。两处时间定义不一致：

- 上一帧计算和提交的耗时被从运动预算扣除。目标推进 5 ms，上一帧耗时 1 ms 时只剩 4 ms 预算。
- 控制线程迟到时，目标可能推进 10–20 ms，预算仍上限 5 ms。

此外，即使修正时间，任意末端目标与瞬时关节速度上限也不一定同时可满足。接近伸直时，小量末端移动可以要求明显的肘关节变化。继续扩大冗余角搜索无法改变这个可行性问题。

### 完整链路离线证据

使用双手柄六轴合成输入 → 项目原始映射、滤波、插值 → 执行器 → 固定版本真实 SDK；源为 90 Hz、名义控制为 200 Hz，并注入接收抖动、迟到和计算耗时。没有连接设备。

320 组回放中，新实现失败 92 次；其中 91 次的同一目标和参考关节用原 SDK 可解。67 次在按真实目标采样间隔给预算后可解。8 条原算法完整完成 4 秒的轨迹，新算法因定时问题提前暂停。其余组中也有真正越出工作空间的轨迹，不能将全部失败计为回归。

对 92 个失败样本在局部冗余角范围内分别做 4001 点密扫，没有找到被原搜索漏掉的可行解。89 个样本的 J4 已超过人工步长界，而 SDK 的冗余角改变不影响该目标下的 J4。

三个最小案例保存在 [teleop_ik_reproduction.json](teleop_ik_reproduction.json)，均为合成输入，不是用户实机记录：

| 情形 | 目标间隔 | J4 所需变化 | 上轮允许变化 | 结果 |
| --- | --- | --- | --- | --- |
| 迟到一个周期 | 10 ms | 0.909758° | 0.9° | 新增拒绝；相同目标用 10 ms 预算可解 |
| 上一帧耗时 1 ms | 5 ms | 0.730879° | 0.72° | 新增拒绝；相同目标用 5 ms 预算可解 |
| 接近伸直、理想定时 | 5 ms | 0.936597° | 0.9° | 几何可达，但瞬时速度要求确实超过给定界限 |

这里的 1 ms 是确定性注入耗时，不是对用户机器的测量。原生日常微基准约 0.059 ms/次，也不能据此排除整条链路的调度和提交延迟。

## 2. Wuji 官方确有天机实现，但不能照搬默认值

审查仓库 [wuji-technology/wuji-hand-teleop](https://github.com/wuji-technology/wuji-hand-teleop)，固定提交 `647801345a6a27dec5cbf56280ce63bb8b2f6a32`。该仓库已归档；没有找到与本次故障完全对应的已解决 issue。

| 对比 | 官方 HTC | 官方 PICO world | 本项目恢复后的行为 |
| --- | --- | --- | --- |
| IK 参考关节 | 当前关节反馈 | 上次成功命令 | 上次接受命令；接合取实测角 |
| 冗余方向 | 上臂追踪器给出参考平面 | 参考平面优先，失败再试 NEAR_REF | NEAR_REF |
| 更新频率 | 120 Hz | 90 Hz | 200 Hz，输入另行插值 |
| 普通 IK 无解 | 跳过该臂该帧 | 跳过该臂该帧 | 双臂协调暂停，显式恢复 |

依据是实际 [HTC 控制器](https://github.com/wuji-technology/wuji-hand-teleop/blob/647801345a6a27dec5cbf56280ce63bb8b2f6a32/src/output_devices/tianji_output/tianji_output/tianji_chest_driver.py#L430-L495) 和 [PICO 求解与发送路径](https://github.com/wuji-technology/wuji-hand-teleop/blob/647801345a6a27dec5cbf56280ce63bb8b2f6a32/src/output_devices/tianji_world_output/tianji_world_output/cartesian_controller.py#L338-L419)。官方不会每次无解都要求重新接合，因此“没有暂停提示”不等于不存在无解。它的速度比例设置也不能证明控制器会怎样执行任意突变目标。

当前只采集头显和手柄；官方的动态肘平面依赖额外上臂追踪器。其 `zsp_para` 是 SDK 的参考方向约定，不应直接解释为手柄姿态或肘的位置向量。[官方方向计算](https://github.com/wuji-technology/wuji-hand-teleop/blob/647801345a6a27dec5cbf56280ce63bb8b2f6a32/src/input_devices/pico_input/pico_input/incremental_controller.py#L259-L337)

### 静止目标也会大幅换解

用本项目 SDK，取当前左臂初始角 `(30, −60, −34, −52, 30, 12, 4)`，令目标严格等于其 FK：

| 策略 | 最大关节变化 | 结果 |
| --- | --- | --- |
| 官方 PICO 默认方向 `(0, −0.5, −0.3)` | 31.347° | SDK 认为合法 |
| 官方 HTC 默认方向 `(0, −1, −0.5)` | 35.692° | SDK 认为合法 |
| 按当前肩关节捕获一致参考方向 | 约 0.00002° | 仅数值差异 |

PICO 此时输出约 `(39.8199, −70.5485, −65.3472, −52, 58.9653, 3.1242, −5.0886)`。目标完全不动，关节仍大幅变化；由于解合法，“失败再回退”不能阻止它。

捕获当前方向只验证了接合时一致性，尚未证明在用户连续横移和转腕过程中不会再碰限位。已经内收后捕获当前姿态，也不会自动变成外展姿态。

官方模型也不同：例如其 J4 为 `[-140°, 78°]`，本项目为 `[-145°, 60°]`，腕部耦合系数也有差异。不得用官方配置扩大本机运动范围。[官方模型](https://github.com/wuji-technology/wuji-hand-teleop/blob/647801345a6a27dec5cbf56280ce63bb8b2f6a32/src/output_devices/tianji_output/tianji_output/config/ccs_m6.MvKDCfg#L1-L13)

## 3. 论文和成熟开源方案解决的是什么

- [DROID 的 IK 控制器](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_ik/robot_ik_solver.py#L23-L100) 使用 DM Robotics 的受约束笛卡尔速度优化，配置关节位置、速度限制及姿态偏好。它允许暂时跟踪误差，不强迫任意目标当帧精确实现。
- [Pink](https://github.com/stephane-caron/pink/blob/c848c85fbac6790bb752a224cc6c86720d986186/pink/solve_ik.py#L152-L240) 明确区分尽量满足的任务代价与必须满足的约束；更换求解器本身不能使冲突的硬约束变得可行。
- Flacco、De Luca、Khatib 的 [SNS 论文（T-RO 2015）](https://iris.uniroma1.it/retrieve/e383532a-4954-15e8-e053-a505fe0a3de9/Flacco_Postprint_Control-of-Redundant_2015.pdf)，DOI `10.1109/TRO.2015.2418582`，在硬关节约束下利用冗余，能力不足时缩放任务速度并保持方向；有 [Rethink Robotics 实现](https://github.com/RethinkRobotics-opensource/sns_ik/blob/354be0f3874a1ac93d2420b0812f26d107c8dd84/sns_ik_lib/include/sns_ik/sns_vel_ik_base.hpp#L61-L118)。

明确判断：应先用实测故障回放判断需要的是冗余姿态策略修正，还是受约束的任务推进。如果是后者，应在控制器中明确表达跟踪误差、关节限位和姿态偏好；不能继续把“精确目标无解”简单包成另一层重试或加大限值。

## 4. 本轮变更与验证

- 删除上轮连续 IK 原生接口、冗余搜索、Python 步长 API 和错误时基，恢复原来的运动学求解路径；不是仅调大阈值。
- IK 出错时附完整末端目标、上一接受关节参考、SDK 输出及标志、SDK/model 版本；执行器附最新实测关节和反馈时间、命令和源样本标识、目标时限。仅失败时格式化到终端，不自动写记录文件。
- 新增纯离线 `scripts/analyze_tianji_ik.py`：仅加载运动学库，对比本项目 NEAR_REF 与官方两组参考方向；可选扫描冗余角，报告关节变化、限位标志和 FK 误差，不发送关节命令。
- 新增真实 SDK 时序回归：双臂六轴输入、90 Hz 源、5/10/20 ms 命令间隔和 0/3 ms 提交耗时，三个预固定 seed 共六个场景。恢复后的实现通过；相同测试可检出保存的上轮实现。

独立端到端验证：用本地真实 SDK 产生左 J6、右 J6、左 J5 三条越限诊断，再按终端格式解析回放，SDK 标志全部匹配，关节结果差为 0°。这里“真实 SDK”指实际厂家运动学程序，不是实机运动记录。

先前单次漏帧使 Quest 状态长期不可用、采集就绪发布顺序及时间缺口锁存的修正保留。Wuji 编码器告警没有造成本次 `step bounds` 暂停的证据；ADB 返回码 `−2` 表示进程被 SIGINT 终止，可由 Ctrl+C 触发，不能单凭它认定 USB 断线。

离线复现命令（项目根目录）：

```bash
conda activate bimanual-teleop
python scripts/analyze_tianji_ik.py docs/teleop_ik_reproduction.json
python scripts/analyze_tianji_ik.py failure.txt --scan-nsp
```

`failure.txt` 可以只包含终端的 `[IK诊断] {...}` 整行，也可包含周围日志。该快照能重放单次选解，但不能重建缺失的整段人体/机器人运动；候选合法不等于连续轨迹已经验证，更不包含碰撞检查。

现场仍缺原始内收动作的完整故障快照与连续轨迹。当前不声称已根治，也不将离线通过作为再次扩大实机运动范围的依据。
