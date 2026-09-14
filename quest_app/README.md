# Quest OpenXR 客户端

Quest 运行原生 OpenXR APK，Ubuntu 通过 USB ADB 逐帧接收。客户端不使用 Unity，不含机器人控制、手形、视频或交互界面；保留空白 XR 场景，以维持运行时正常帧循环。

左右 Touch Plus 按物理左右手安装。每个 XR 更新周期以同一查询时间读取左 grip、右 grip 和头显位姿，并请求 90 Hz。Quest 3 / 3S 面向同一 APK，但两款设备的颈挂追踪与持续运行仍需分别实测。

## 构建

依赖版本固定如下：JDK 17、Gradle 8.5、Android Gradle Plugin 8.1.4、Android SDK 32、Build Tools 33.0.1、NDK 27.0.12077973、CMake 3.22.1、OpenXR Android loader 1.1.53。目标 ABI 为 arm64-v8a，最低 Android API 26。

用 Android 命令行工具安装固定组件：

```bash
sdkmanager 'platform-tools' 'platforms;android-32' 'build-tools;33.0.1' 'ndk;27.0.12077973' 'cmake;3.22.1'
```

设置 JDK 17 的 `JAVA_HOME` 和 Android SDK 的 `ANDROID_HOME` 后，在本目录运行：

```bash
./gradlew assembleDebug
```

构建输出为 `build/outputs/apk/debug/QuestCapture-debug.apk`；交付副本为 `artifacts/quest-capture-debug.apk`。首次构建下载固定提交的 Meta OpenXR SDK，源码归档及 Gradle 分发包均校验 SHA-256。可复用已经解压的同版本 SDK：

```bash
./gradlew assembleDebug -PmetaSdkSource=/path/to/meta-openxr-sdk-v85
```

外部目录必须对应提交 `bbed2f20e38a5df7113630771c83cb8279e4fc26`；该选项用于复用构建缓存，不切换 SDK 版本。项目只拥有采集器，直接链接官方 SampleXrFramework，不复制或修改其源码。

## 安装与独立检查

头显先开启开发者模式、连接 USB，并允许此电脑调试。`SERIAL` 使用 `adb devices` 返回的 USB 设备序列号。安装交付 APK：

```bash
adb -s SERIAL install -r artifacts/quest-capture-debug.apk
```

在一个终端先启动接收：

```bash
adb -s SERIAL logcat -v raw -T 0 QuestCapture:I '*:S'
```

在另一个终端启动 APK，每次启动传入新的 32 位 UUID hex：

```bash
adb -s SERIAL shell am force-stop org.bimanual.questcapture
adb -s SERIAL shell am start -n org.bimanual.questcapture/.MainActivity --es session_id 0123456789abcdef0123456789abcdef
```

示例 session ID 仅用于独立检查。项目 Python 接收器负责生成新 ID，并按 ID 过滤该次运行；正常使用不要复用示例值。直接从头显启动时，APK 自行生成 UUID。

颈挂前需确保摘下头显后 XR 会话仍活跃。应用的 keep-screen-on 不能替代系统的佩戴检测设置；需要按实际 Horizon OS 配置开发模式下的持续运行，并检查输出 session state。系统菜单、失焦、休眠或摘下行为不能被客户端伪装成有效追踪。

## 原始协议 v1

每条消息是一行 JSON，logcat tag 为 `QuestCapture`。原生端保留 **OpenXR 原始右手坐标：X 向右、Y 向上、Z 向后，位置单位米，四元数顺序 xyzw**。Python 接入层统一换成项目的 X 向前、Y 向左、Z 向上。

帧消息字段：

| 字段 | 语义 |
| --- | --- |
| `v`, `type`, `session` | 协议版本 1、`frame`、本次 APK 运行 ID |
| `seq` | 从 0 递增的源帧序号；事件不占用帧序号 |
| `origin` | 本帧使用的 LOCAL 参考系版本，初始为 0 |
| `query_ns` | Quest 的 `CLOCK_MONOTONIC` 查询时间，纳秒 |
| `xr_time` | 同一时刻经 `XR_KHR_convert_timespec_time` 转换的 XrTime |
| `send_ns` | 写入日志前的 Quest 单调时间，纳秒 |
| `state` | 原始 `XrSessionState` 整数 |
| `refresh_hz` | 运行时报告的实际显示刷新率 |
| `head`, `left`, `right` | 下述位姿对象；左右使用 grip space，头显使用 VIEW space |

位姿对象是 `{"p":[x,y,z],"q":[x,y,z,w],"flags":15,"active":true}`。`flags` 保留原始 `XrSpaceLocationFlags`；位置或朝向无效时，仅对应的 `p` 或 `q` 写 `null`。`active` 是手柄 pose action 的原始有效状态；头显为 `null`。valid 与 tracked 是不同标志，不能只看 `active` 或非空位姿判断是否可控制机器人。

三个 `xrLocateSpace` 使用同一当前查询时间和 `LocalSpace`，不乘头显位姿的逆，也不读取 SDK 默认的未来显示时刻预测位姿。这里提供的是**查询时刻的运行时估计**，不声称是底层相机或 IMU 的原始测量。Quest 时钟与电脑时钟不同，不能直接相减；主机另行标记接收时间。

LOCAL 空间在头显运动时保持固定，但运行时重定位或用户 recenter 可能改变它。`reference_space_change` 事件到达时立即处理；只有查询时间达到事件的 `change_time`，后续帧才采用新 `origin`。事件不代表已完成机器人坐标标定。

`origin` 是参考系的唯一标识，不用数值大小推断生效顺序。Python 额外发布 `quest.origin_changed`，表示首次接收到新参考系的帧；它不替代原始事件的精确生效时间。缺少对应原始事件会报告元数据缺口。

事件对象包含 `v`, `type="event"`, `session`, `event`, `device_ns`, `details`：

| `event` | `details` |
| --- | --- |
| `reference_space_change` | `change_time`、新 `origin`、`pose_valid`、`pose_in_previous_space` 的 `p/q`；变换无效时二者为 null |
| `session_state` | 原始 `state` 与 OpenXR `time` |
| `refresh_rate` | `from_hz` 与 `to_hz`；初始化的 `from_hz=0` 表示尚无前值 |
| `error` | 失败的 `operation` 与原始 OpenXR `result` |

90 Hz 是向运行时提出的请求，实际节奏由 XR 帧循环决定；降频或帧延迟如实呈现，不通过重复帧补齐。logcat 缓冲可能丢帧，Python 在逐行接收时检查源序号并报告缺口。这里不实现重传、补帧或自动重启。

Python 的 `health()` 在超过 1 秒没有收到新帧时报告收流停顿；这只检查主机收流活动，不证明最近收到的帧没有在日志缓冲中积压。有效性、追踪标志、action active 和 XR 会话状态分别保留。

实时查看左右手柄和头显位姿，在项目根目录运行 `python scripts/view_quest.py`。查看器不保存运行记录；追踪丢失、参考系变化和收流停顿由接收端实时检测。

## 验收

- APK 可构建、安装；左右及头显各自包含正确位姿和有效性标志。
- 静置手柄、移动头显时，LOCAL 中手柄位置保持稳定；运动轴经 Python 换基后对应前、左、上。
- Quest 3S / 3 各颈挂连续采集 30 分钟，检查源帧间隔、实际刷新率、序号缺口、主机接收间隔与积压现象；不能用 getter 调用频率声称采集达 90 Hz。
- 单侧遮挡、原点重置、系统菜单、USB 断开与恢复能被主机识别；颈挂翻腕、交叉及静置不按键时分别测试。

构建通过仅验证工程与 API 链接，不代表上述硬件行为已经通过。交付构建和自动检查记录见 [验收记录](artifacts/VERIFICATION.md)。
