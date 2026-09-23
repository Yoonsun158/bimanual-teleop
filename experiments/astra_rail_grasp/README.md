# Astra 导轨抓取实验

由当前对话查看相机图像、读取反馈，再提交一个有界动作。模型推理不在控制循环内；本目录不调用 OpenAI API。代码复用项目的 Tianji、Wuji 驱动和离线运动学，原遥操作入口不变。

本实验的目标是抓住桌面金属导轨、试抬并累计抬高约 20 mm、保持 5 秒、沿原升降路径放回，确认桌面支撑后释放。左臂不可用时，启动仅右臂、右手的会话。不能在夹持过程中自动移除故障侧或切换模式。

## 环境与启动原则

所有命令在当前工作树根目录执行，使用 Conda `bimanual-teleop`：

```bash
conda activate bimanual-teleop
cd /home/yuchen/.codex/worktrees/astra-rail-grasp/bimanual-teleop
python -m experiments.astra_rail_grasp --help
```

运行器独占设备连接；客户端只连接该运行器。跨工作树设备锁与已有控制进程检查用于避免同时控制同一设备。仿真不会连接 SDK 或硬件。纯观察启动不清错、不改控制参数、不使能、不回预设姿态；只有显式进入运动会话后才接管当前实测姿态。

只读查看真实右臂、右手及相机：

```bash
python -m experiments.astra_rail_grasp --run-dir experiments/astra_rail_grasp/runs/right-observe serve --backend hardware --side right
```

另一个终端使用相同 `--run-dir` 调用客户端：

```bash
python -m experiments.astra_rail_grasp --run-dir experiments/astra_rail_grasp/runs/right-observe call status
python -m experiments.astra_rail_grasp --run-dir experiments/astra_rail_grasp/runs/right-observe call observe
python -m experiments.astra_rail_grasp --run-dir experiments/astra_rail_grasp/runs/right-observe call shutdown
```

每次独立试验使用一个新的运行目录。仿真运行器用 `serve --backend fake --side right --allow-motion`；实机需要 `serve --backend hardware --side right --allow-motion`。`--allow-motion` 只开放后续 `engage` 入口，启动本身仍只读。双臂会话用 `--side both`；左臂有故障时先关闭只读会话，再明确启动 `--side right`，不清除左臂错误，不向左手发目标。

## 客户端接口

`call OP --args 'JSON'` 或 `call OP --args-file PATH` 提交参数。较长参数使用文件。除只读查询外，可指定 `--request-id ID`；不确定上次调用是否成功时使用**同一个 ID、相同参数**重试。动作结果中的 `action_id` 只表示接受，后续通过 `status` 的 `action` / `recent_actions` 查看实测到位结果。

| 操作 | 参数与作用 |
| --- | --- |
| `status` | 状态、当前动作、反馈、相机健康；不拍摄记录图像。 |
| `observe` | 保存当前 RGB、可用深度及 JSON，返回 `observation_id` 和图像绝对路径。只有 `usable_for_action=true` 才可用于后续判断。 |
| `engage` | `observation_id`、`note`、`onsite_ready:true`；从实测姿态接管选中设备。 |
| `arm-step` | `observation_id`、`note`、`moves`，例如 `{"right":{"translation_mm":[0,10,0],"rotation_deg":[0,0,0]}}`；`near` 默认 `true`，接近时非零平移固定为 10 mm。明确授权的空载定位使用 `near:false`，允许 10–200 mm。 |
| `hand-step` | `observation_id`、`note`、`side`、`purpose`（`prepare` / `grasp` / `release`）、`joints_deg`，例如 `{"finger1_joint1":1}`，角度为增量。 |
| `mark` | `observation_id`、`note`、`what`：`direction-verified`（另填 `side`）、`grasped`、`supported`、`released`。这些是根据图像作出的判断，不是自动识别。 |
| `lift` | `observation_id`、`note`、`mm`（只能为 `10` 或 `-10`）；正数上升，负数沿最近一段已完成的上升路径下降。 |
| `pause` | 可选 `reason`；终止当前分段动作并维持固定目标。 |
| `resume` | 新的 `observation_id`、`note`；健康状态恢复后允许新动作，旧动作不会继续。 |
| `shutdown` | 无参数；带载或未结束动作时拒绝，成功后返回停机确认。 |

运动和视觉标记均消耗观测令牌，下一步必须重新 `observe` 并检查图像。每次 `note` 记录实际视觉判断，不能用固定文案代替查看现场。暂停中断的升降高度会标为不确定，禁止自动恢复升降或标记落稳；需要现场接管。正常退出被拒绝时，关闭客户端或按 Ctrl-C 不会绕过载荷保护。

## 动作约束

- 机械臂以 200 Hz、灵巧手以 120 Hz 独立续发目标，单次底层命令有效期最多 50 ms。图像保存、日志写入和客户端等待不负责维持该循环。
- 机械臂初始速度及加速度比例均为 5%，平移速度最多 5 mm/s。按最新操作要求，每段非零平移至少 10 mm（1 cm）；本次右臂空载定位为先向下 100 mm、重新观察后再向后 200 mm，接近导轨时仍固定为 10 mm。远距上限 200 mm 只是软件限幅，不代表路径安全。旋转仍最多 1°。若可见间隙不足以完成整段位移，就停下，不用更小平移绕过要求。
- 手部使用 `kp=3`、`kd=0.05`、每关节电流上限 0.5 A。单步最多 1°，过渡不少于 0.3 秒。全零角度不代表张手，所有手形从实测角度逐步确认。
- 同步升降每步为 10 mm，先试抬 1 cm，确认稳定后再抬 1 cm，累计最多 20 mm；首先空载验证左臂 `−Y`、右臂 `+Y` 是否确为上升。每侧基座坐标不同，不能复制同一平移向量。
- 每次只接受一个动作；动作必须绑定本会话的新观测并有未过期的请求期限。重复请求返回原结果，不重放动作；新运行器不恢复上次动作。

电流绝对值达到 0.4 A 持续 0.2 秒，或手部过渡结束 0.5 秒后跟随误差仍超过 5°，停止继续闭合。电流不是夹持力。双臂升降差或横向漂移超过 1 mm、朝向变化超过 0.5°并持续 0.1 秒时暂停双方。

正常暂停会固定当前目标并继续发送，客户端断开不会松手。反馈故障走驱动故障停机路径；此时不保证能保持物体。可能带载时拒绝正常退出；先确认导轨落稳并释放，再正常退出和检查停机反馈。

## 现场操作

1. 先观察导轨、手指、桌面和运动路径，确认现场人员能操作实体急停。视野或反馈不足时停止推进。
2. 执行本次用户指定的右臂空载定位：先向下 10 cm，到位并重新观察后再向后 20 cm，不把两段合并成斜线运动。以已验证的右臂基座方向表达，分别是 `translation_mm:[0,-100,0]` 和 `[-200,0,0]`，均设置 `near:false`。每次只有整段路径可见且间隙足够才发动作；仍需观察实际方向，并确认导轨可由单手稳定支撑。按当前平滑插值及速度限制，两段名义时间分别为 30 秒和 60 秒，到位还需实测反馈确认。
3. 逐手、逐关节确认张手方向，记录本次张手与预抓姿态。每段动作到位后读取新图像；旋转之前检查手指扫过的空间。
4. 以每段 1 cm 接近并逐步夹持，空间不足则停下。确认抓取后固定朝向，先试抬 1 cm；确认导轨跟随且未滑动、卡住，再抬 1 cm。
5. 到约 20 mm 后保持至少 5 秒，逐段沿保存路径下降。确认桌面支撑，再逐步张手释放并退出。

不做完整手眼标定意味着没有可靠的相机到机械臂坐标变换。小步视觉核对只能减少误差，不能提供完整路径碰撞验证。金属表面深度缺失或异常时按未知处理，不填零距离、不用邻近桌面深度代替导轨高度。

## 记录与验收

运行记录保留图像时间戳、请求及动作编号、目标与实测状态、手部电流、暂停或故障原因。记录目录由本实验自己的 `.gitignore` 排除。

仿真测试验证接口边界和故障行为；它不证明实际抓取稳定性或碰撞安全。只有现场图像及反馈支持抓住、抬起、保持、放回和释放全部步骤，才能记为抓取完成。中途停止时记录实际完成阶段和停止原因。

运行本实验的离线测试：

```bash
conda run -n bimanual-teleop python -m unittest discover -s experiments/astra_rail_grasp/tests -v
```

测试不连接硬件。运行目录中的 `events.jsonl` 由独立日志线程写入，`images/` 保存图像与深度，`observation-*.json` 保存该次视觉判断对应的状态。
