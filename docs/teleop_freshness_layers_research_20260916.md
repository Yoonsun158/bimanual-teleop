# 输入新鲜度、迟到目标与 watchdog 的分层调研

调研日期：2026-09-16。检查对象是当前工作区及下列作者公开仓库的固定提交；公开仓库可能包含论文发表后的改动。只读取外部源码，未运行外部控制程序、连接设备或发送运动命令。下文明确区分源码行为和本项目的设计建议。

结论：**输入失效与恢复策略由遥操作控制层负责；目标有效期由执行层传递并复查；设备驱动及原生发送层负责拒绝过期命令；命令断流的最终保护应在独立于上层计算的设备侧执行环节。** 不应把所有检查合并到 UI、mapper 或某一个通用超时函数中。当前分层方向基本正确，主要缺口是主机 watchdog 的运行独立性，以及尚未确认的控制器端断流行为。

## 论文代码与工程参考

### Diffusion Policy

[论文项目页](https://diffusion-policy.cs.columbia.edu/)，代码提交 `5ba07ac6661db573af695b419a7947ecb704690f`。

- 环境的 `exec_actions` 仅保留执行时间晚于接收时刻的动作，再交给机器人进程排程：[real_env.py:310–339](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/diffusion_policy/real_world/real_env.py#L310-L339)。这是动作调度入口的迟到过滤。
- 控制进程把目标时间转换为本机单调时间，在高频循环中更新插值轨迹：[rtde_interpolation_controller.py](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/diffusion_policy/real_world/rtde_interpolation_controller.py)。`schedule_waypoint` 收到 `time <= curr_time` 时直接返回原轨迹；轨迹取样超出末端时夹到终点：[pose_trajectory_interpolator.py](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/diffusion_policy/common/pose_trajectory_interpolator.py)。因此，在已检查的控制路径里，丢弃迟到 waypoint 不会自动触发上层输入超时停止。
- 评估程序预留 10 ms 的动作执行时间；整批动作都迟到时，取最后一个动作并重新安排到下一个时间格：[eval_real_robot.py:322–337](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/eval_real_robot.py#L322-L337)。这属于策略评估的恢复选择，不能当作遥操作输入新鲜度保证。

### Universal Manipulation Interface（UMI）

[论文项目页](https://umi-gripper.github.io/)，代码提交 `d095ba9590df789df5189eea5ee7e431689038a6`。

- 双臂环境先过滤 `timestamps <= receive_time`，然后按机器人和夹爪分别减去配置的动作延迟来排程：[bimanual_umi_env.py:481–514](https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/umi/real_world/bimanual_umi_env.py#L481-L514)。这里的时间是期望执行时间，不是观测有效期；延迟补偿也不是给旧输入续期。
- 插值器在消费端再次忽略已经错过的 waypoint：[pose_trajectory_interpolator.py:105–134](https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/umi/common/pose_trajectory_interpolator.py#L105-L134)。上层过滤后仍可能经过 IPC、排队和调度，所以消费端复查有独立作用。
- 双臂评估程序同样存在“整批迟到后重新排程最后动作”的分支：[eval_real_bimanual_umi.py:559–576](https://github.com/real-stanford/universal_manipulation_interface/blob/d095ba9590df789df5189eea5ee7e431689038a6/scripts_real/eval_real_bimanual_umi.py#L559-L576)。不建议复制到本项目的 Quest 断流处理。

### Deoxys（VIOLA 等研究使用的底层控制库）

[仓库及其 VIOLA 引用说明](https://github.com/UT-Austin-RPL/deoxys_control)，代码提交 `97396fd91324e9e961f061544e80a208889526ff`。

- 机器人侧 C++ 订阅器设置 ZeroMQ `conflate=1`，只保留最新消息，避免逐个执行积压的即时目标：[zmq_utils.cpp:18–24](https://github.com/UT-Austin-RPL/deoxys_control/blob/97396fd91324e9e961f061544e80a208889526ff/deoxys/franka-interface/src/utils/zmq_utils.cpp#L18-L24)。最新消息仍可能来自过旧输入，因此该机制不能代替源时间检查。
- 在控制开始后，订阅线程连续 20 次接收不到消息会设置 `running=false`、`termination=true`：[franka_control_node.cpp:320–346](https://github.com/UT-Austin-RPL/deoxys_control/blob/97396fd91324e9e961f061544e80a208889526ff/deoxys/franka-interface/src/franka_control_node.cpp#L320-L346)。示例配置 `POLICY_RATE=20`、非阻塞接收，对应名义约 1 秒，实际还受线程调度影响：[charmander.yml](https://github.com/UT-Austin-RPL/deoxys_control/blob/97396fd91324e9e961f061544e80a208889526ff/deoxys/config/charmander.yml)。这是按未收到消息次数计的断流处理，不是严格的单条命令绝对期限。
- Franka 控制回调观察 `running` 并返回 `MotionFinished`：[joint_pos_callback.h](https://github.com/UT-Austin-RPL/deoxys_control/blob/97396fd91324e9e961f061544e80a208889526ff/deoxys/franka-interface/include/utils/control_callbacks/joint_pos_callback.h)、[torque_callback.h](https://github.com/UT-Austin-RPL/deoxys_control/blob/97396fd91324e9e961f061544e80a208889526ff/deoxys/franka-interface/include/utils/control_callbacks/torque_callback.h)。可借鉴的是“机器人侧检测，控制回调执行退出”的职责划分，不是其超时数值或其他模式的停止动作。

### GELLO

[论文项目页](https://wuphilipp.github.io/gello_site/)，代码提交 `204f53a64bef89471a1e483b0f874f755fbd2d3a`。本次检查的是 Python/ZMQ/UR 路径，不涵盖所有机器人后端或 ROS 2 路径。

`RobotEnv.step` 直接下发关节目标；ZMQ 服务端设置 1000 ms 接收超时，但捕获超时后只继续循环：[env.py](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/gello/env.py)、[robot_node.py:30–61](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/gello/zmq_core/robot_node.py#L30-L61)。这说明“存在 socket timeout”本身不意味着机械臂会停止。该路径未提供本项目这种贯穿输入、目标和发送边界的有效期；不能据此推断厂商控制器自身没有保护。

### MoveIt Servo（工程参考，并非上述论文的实现）

代码提交 `822acad0f32dd2caded0930e77d176d435b1ac98`。Servo 节点比较当前时间和输入消息的 `header.stamp`，过期后进入 `smoothHalt`，逐步生成停止状态。关节、速度和位姿输入都有该处理：[servo_node.cpp:241–330](https://github.com/moveit/moveit2/blob/822acad0f32dd2caded0930e77d176d435b1ac98/moveit_ros/moveit_servo/src/servo_node.cpp#L241-L330)。默认 `incoming_command_timeout=0.1` 秒：[servo_parameters.yaml:102–108](https://github.com/moveit/moveit2/blob/822acad0f32dd2caded0930e77d176d435b1ac98/moveit_ros/moveit_servo/config/servo_parameters.yaml#L102-L108)。可借鉴 Servo 层处理输入中断的方式；这里采用 ROS 时钟，不能把它的时间戳直接与 Quest 的设备单调时钟相减。

## 当前项目的职责与实现

| 检查对象 | 当前实现 | 建议归属 |
| --- | --- | --- |
| Quest 输入中断、追踪丢失、乱序、参考系变化 | `QuestInputMonitor`，100 ms；故障锁存，恢复须重新接合 | `control/arm` 的输入监测与运行状态机；设备适配器负责原始时间、序号、事件 |
| 源数据积压 | 同时看主机到达时间、源查询时间和历史最小差值 | 输入边界；此估计只表示相对最小传输延迟新增的积压，不是绝对单向延迟或时钟同步 |
| 插值造成旧输入重新包装 | `PoseGoalInterpolator` 对所用源帧取最早截止时间，再限制为当前时刻后 50 ms | 映射/插值层保留有效期，不负责实际停机 |
| IK/伺服计算后目标已过期 | `TianjiCartesianExecutor.submit` 在计算前后复查 | 执行层 |
| 命令提交超期、上一有效目标到期、反馈序号不再推进 | `TianjiDriver`，默认 50 ms watchdog；周期性线程请求 hold | 驱动拥有命令和反馈健康责任；周期性最终动作宜下沉到独立执行环节 |
| SDK 等待发送后包已过期 | C++ `target` 等待前后检查；`tj_hook_send` 在 `sendto` 前检查，过期则报告未发送 | 原生发送边界，必须保留 |
| 双臂/手部一起暂停，恢复时重建参考 | 各 runtime 与 `CombinedTeleop` 协调，带取消代际 | 系统运行状态机 |

本地代码入口：

- [QuestInputMonitor](../bimanual_teleop/control/arm/quest.py)，尤其 `try_publish` 和 `current`。
- [PoseGoalInterpolator](../bimanual_teleop/control/arm/mapping.py)，尤其 `sample`。
- [TianjiCartesianExecutor](../bimanual_teleop/control/arm/cartesian.py)，尤其 `submit`。
- [TianjiDriver](../bimanual_teleop/devices/tianji/driver.py)，尤其 `_watch`、`_send_command`、`request_hold`。
- [C++ bridge](../tianji_bridge/bridge.cpp)，尤其 `target`、`tj_hook_send`。
- [WujiProcess](../bimanual_teleop/control/hand/process.py) 保留源数据和工作线程的截止时间，并检查父进程心跳；[Wuji 适配器](../bimanual_teleop/devices/wuji/adapter.py) 在 `submit` 检查过期。已检查的手部路径没有展示与 Tianji 等价的原生发送时有效期检查，不能把两者当作相同保证。手部暂停时维持已选保持姿态属于显式 hold 策略，应与继续跟随旧输入区分。

当前的“迟到处理”通常会升级为暂停：executor 拒绝目标、驱动发现命令过期、runtime 捕获异常都会进入 hold/pause 路径。它与 UMI 对单个迟到 waypoint 的静默跳过有实质区别。

## 建议采用的边界

```text
设备接收：原始序号、时间、有效标志
    ↓
输入监测 / runtime：判断输入是否有效，建立源有效期，管理暂停和重新接合
    ↓
映射 / 插值 / executor：传递有效期；计算前后复查；旧结果不能跨接合代际生效
    ↓
驱动 / C++ 发送：只接收仍有效命令；队列不补发过期目标；发送前最后复查
    ↓
独立执行循环 / 控制器：命令断流或反馈异常时执行适合当前模式的停止策略
```

同一条命令只传一个明确的绝对截止时间，下游可以收紧，不能延长：

`发送截止时间 = min(所用活动源帧的有效期, 目标生成时间 + 命令寿命上限, 驱动接受时间 + 驱动上限)`

本项目中，插值器已按目标生成时刻增加 50 ms 上限；驱动再次取最小值不会重新给旧目标完整 50 ms。参考锚点的历史样本用于定义坐标关系，不应简单把所有历史 `source_refs` 都当作必须持续新鲜的活动输入。保持命令也应由暂停策略显式生成，不能伪装成新的操作者输入。

“期望执行时刻”和“最晚允许发送/执行的时刻”是不同概念。当前即时遥操没有必要照搬 UMI 的未来轨迹排程；以后接入 action chunk 时，应显式增加计划执行时间，并保留独立有效期。跨机器转发时需要明确时钟转换或本地租期协议，不能直接传递另一台机器的 monotonic 数值作为本机期限。

优先改进项：

1. **保留现有分层检查，把最终 watchdog 的运行独立性补齐。** 当前 `_watch` 是 Python 线程，和提交路径竞争同一把锁；上层长时间占用 GIL 或锁时不能保证按 50 ms 准时执行。原生发送钩子只拦截未发送的包，不会撤回已接收目标。应优先核实天机控制器的原生通信 watchdog/停止语义；主机侧可增加不依赖 Python 回调的 C++ 周期性检查或独立控制进程，并限制其锁等待。C++ 仍不能覆盖整个主机掉电、进程被杀或网络断开，最终需要设备侧机制。
2. **统一超时参数的含义和来源。** 将输入新鲜度、目标寿命、反馈新鲜度、启动等待分别命名、配置并记录，避免 Quest 的 100 ms 和目标的 50 ms 在多个位置独立维护。不同故障仍应保留各自计时，不能混成“任何活动都能喂”的一个 watchdog。保留计算后和实际发送前的复查，数据类型继续只承载字段。
3. **先保留当前超期暂停策略。** 若以后减少非必要暂停，只允许丢弃已被更晚的新鲜目标替代的旧排队项；输入超过有效期、上一有效命令已到期、反馈中断或参考系改变仍应暂停并显式重新接合。恢复数据不能抹掉已经发生的失效区间。不要照搬整批迟到后给最后动作改时间戳的策略。
4. **超时阈值由实测延迟和允许的失控运动距离决定。** 200 Hz 的调度周期是 5 ms，50 ms 命令期限允许一定调度抖动，两者不是同一要求。端到端停止还包括检测延迟、发出停止请求的延迟和机械制动时间；当前 50 ms 不能解读为机械臂必在 50 ms 内停稳。之前的[负载测试](teleop_timeout_fix_20260915.md)可作为主机延迟依据，但不代替断流停机实测。

## 验证与边界

- 本次只新增本文，没有修改控制逻辑或超时参数。
- Conda `bimanual-teleop` 下，映射、runtime、watchdog 竞态三组共 79 项通过；采用 discovery 入口后，SDK 离线时序回归另 1 项通过，共 80 项。
- 首次以包名加载时序回归时，其内部顶层测试模块导入失败；改用该目录的 discovery 入口后通过。这是调用入口问题，本次没有改测试文件。
- 测试不打开设备。结论来自静态代码检查和已有离线测试；没有验证控制器断网后的实际动作、物理停止时间或硬件级安全保证。
