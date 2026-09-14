# Tianji 原生桥接

构建固定的 [官方 SDK](https://github.com/cynthia-you/TJ_FX_ROBOT_CONTRL_SDK/tree/02440e886fb59095711eb9ec6dcbedd8be08922a)，提交为
`02440e886fb59095711eb9ec6dcbedd8be08922a`，控制 SDK 版本为 `100343014`。
新代码不导入或链接旧项目目录。只支持 Linux；需要 CMake、C++17 编译器和 Python 3。

先按[根 README](../README.md)创建项目 Conda 环境，再构建：

```bash
conda activate bimanual-teleop
cmake -S tianji_bridge -B tianji_bridge/build -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
cmake --build tianji_bridge/build -j 4
ctest --test-dir tianji_bridge/build -V
```

首次构建联网获取源码；之后使用校验过的缓存。原始源码及其许可证在
`tianji_bridge/build/sdk/<commit>/`，每个文件按固定提交的 Git blob SHA 验证。
补丁只写入 `tianji_bridge/build/vendor/`。产物为 `libtianji_bridge.so` 和
`libtianji_kine.so`，放在同一目录。`models/ccs_m6_40.MvKDCfg` 是该提交的原样
名义模型，来源和许可见 `models/README.md`。导出配置比对可用于诊断，不是运动前必须提供的凭据。

## 接口与时间

ABI 2 定义在 `bridge.h`，采用自然对齐及固定宽度整数。Python 与原生库须一起更新、重新构建。
ABI 2 反馈布局保留原有 PVT 与辨识字段，现有控制流程不使用这些字段。整个进程只有一个设备连接，
同时拥有左右臂；重复 `tj_open` 返回 busy。FK/IK 为本地计算，不需要设备连接。

- `tj_open` 只连接和查询 VERSION，不清错、不使能。
- `tj_configure` 仅写选中臂的工具、K/D、速度比例、固定基座方向与零速度前馈。
  `tj_engage` 将当前关节目标、ImpType=2 和 state=3 合包发送。发送结果与控制器反馈分别判定。
- `tj_move_joints(arm, target_deg[7], velocity_ratio, acceleration_ratio, token, expires_ns)`
  一次发送厂家位置目标、比例和 state=1。`arm` 为 0/1，比例为 SDK 支持的 1–100。
  从其他模式进入位置模式时，只发送当前实测角作为种子；收到新的 state=1 反馈后，
  再通过 `tj_submit` 单独发送最终目标。模式切换会覆盖同包的目标，不能把最终回位目标与首次切模式合包。
  已在位置模式时直接发送最终目标，由控制器生成轨迹；上层不重复发送插值目标。
  `tj_position_mode` 保留旧签名兼容；项目运动入口使用可传比例的 `tj_move_joints`。
- `tj_confirm_cartesian` 不发送报文；确认 SDK 实际反馈为 state=3/ImpType=2 后，结束此前的位置运动停止策略。仅入队或超时的模式切换不会清除该策略。原生库和 Python 绑定须一起构建，ABI 2 数据结构保持不变。
- `tj_reset_emergency` 仅用于操作者已确认释放实体急停后的显式恢复。它检查指定臂处于 state=100/error=13，发送一次官方 `RESET0/RESET1` 并检查返回结果；不使能，也不自动重试。上层读取后续 state=0/error=0 反馈，没有额外速度或持续时长条件。
- `tj_submit` 返回成功仅表示 SDK 已接受待发包。单一待发槽不能覆盖；调用方给出
  严格递增 token 及主机 `CLOCK_MONOTONIC` 到期时间。`tj_submit`／`tj_engage` 遇到前包待发时，
  释放发送互斥锁等待，最多 5 ms 且不超过新目标到期时间；持续繁忙仍拒绝，过期目标不入队。
  实际发送前再次检查到期时间。
- `tj_poll_send` 返回真实 UDP 内容、token、字节数／错误号。`attempted=1` 时 `sent_ns`
  是调用 `sendto` 前的时间；`attempted=0` 时是过期或关闭取消的判定时间。
  成功写入 UDP socket 也不能证明设备执行。token=0 表示 SDK 自身的配置查询流量。
- `tj_hold(mask, token)` 请求选中臂停止。对于本会话位置运动，它直接发送官方
  `RSTA0/RSTA1/RSTA01`，避免低速标志导致 SDK 跳过请求；其返回结果与后续反馈分别判定。
- `tj_close` 只关闭连接，不代替协调暂停；关闭后的队列仍可读完。

反馈在实际 `recvfrom` 返回处获取主机单调时间，每个有效 DCSS 包只入队一次，
不通过反复读取 latest 生成伪样本。SDK 连接初始化时会先清空尚未开始会话的旧 UDP 包。
SDK 不提供设备采样时间；`received_ns` 不应被标注为设备采样时间。
`target` 来自控制器 `m_FB_Joint_Cmd`，实际位置来自 `m_FB_Joint_Pos`。
一次性位置运动使用本次成功发送后的新反馈确认：state=1、low_speed=1，且控制器生成指令已到最终目标。
比较按协议 float32 表示进行，不再给实测角度增加到位误差门槛，也不添加停稳观察时长。

反馈、发送结果分别使用 1024 项队列。队列满时保留已有队列内容，累计报告丢失数量及
首尾包索引；反馈 latest 仍更新。上层需把累计量变化转换为缺口事件。
原生队列是否排空、数据是否有缺口和机器人是否已停止是三个独立状态。

ABI 保留 SDK 单位：关节度／度每秒、工具平移毫米、XYZABC 度、电流千分比、
关节传感器力矩 Nm。外力矩／末端扰动是厂商估计。工具动力学十项和 K/D 保留原值，
不会擅自根据文档中含糊的单位转换。上层负责统一米／弧度与字段语义。
FK/IK 使用各臂基座坐标；位姿矩阵行优先、平移毫米、旋转无量纲；IK 使用
ZSPType=0 和调用方参考关节角，返回奇异及限位标志，不能忽略这些标志直接执行。

## 对厂商源码的必要修改

`prepare_sdk.py` 校验锚点后，重复构建会从原始源码重新生成以下修改：

1. 在 `DoRecv` 两个 Linux `recvfrom` 后加入逐包采集钩子。
2. 在 `OnSetSend` 发布待发包时绑定 token；替换 `DoSend` 的主控制包 `sendto`，
   检查真实结果与到期时间。命令构建／发布／发送使用同一互斥锁。
3. 文件传输的 TCP 接收线程改为可 join；关闭连接后等待线程退出，避免接收对象提前销毁。
   文件状态和连接状态使用原子变量，TCP connect/send 使用 2 秒 socket 超时，
   下载的文件交换最多等待 10 秒；处理短发送并禁止 SIGPIPE 终止进程。
4. 文件下载检查写盘零进展及 `fclose` 错误。只有协议完成、文件非空且无错误时，
   才将 `.partial` 文件重命名为用户指定文件；失败删除临时文件。上层另做完整字段检查。
5. 删除不受 SDK 日志开关控制的 `system version:...` 和 `Robot released` 常规输出；
   版本查询、释放操作和 SDK 错误输出保持不变，不重定向标准输出或标准错误。

仅对 127/8 回环地址，桥接调用 SDK C 包装下的同一个 `CRobot::OnLinkTo`，以便
运行回环测试，同时将接收 socket 绑定回环地址，隔离后台真实控制器 UDP 流量；
其他地址仍使用 SDK 的标准 `OnLinkTo` 和原有监听地址。

测试固定使用本机回环端口 4729／4730 和 10240，不接受真实机器人地址。覆盖逐包收取、
队列溢出、连续提交等待发送、待发超时不覆盖、到期不发、发送错误、合包内容、单臂停止、完整／中断下载与
磁盘写入错误，以及双侧位置回位和停止报文。另打印 100 周期、200 Hz 的提交耗时和实际发送间隔，超期跳过、不补发。
该测试衡量本机 SDK 与 socket 通路，不能替代机器人闭环频率和断线行为验收。
