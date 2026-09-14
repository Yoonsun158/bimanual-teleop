# Quest 模块交付检查

本页保留 2026-09-10 至 11 日的历史构建与实机验收，并记录 2026-09-14 项目目录更名后的 APK 重建。历史构建主机为 Ubuntu x86_64，Python 3.10.12。旧日志、汇总和预览图已保存在项目外 `../../../demo-archive/20260913-module-cleanup/history.tar.zst`（相对于本文件），归档内路径为 `data/`；当前项目不再提供独立采集或诊断落盘入口。

## APK

- 文件：[quest-capture-debug.apk](quest-capture-debug.apk)，8,487,977 字节。
- 包名：`org.bimanual.questcapture`；版本：`0.1.0` / versionCode `1`。
- Android 最低 API 26，target / compile API 32，仅 `arm64-v8a`。
- 使用 Android debug 证书签名，`apksigner verify` 验证通过（v2 签名）。
- SHA-256：`0c7f95c7c12703e45b79e596a1dfb786d1c8d9678cab26790e4263f31fdd925e`；另见 [SHA256SUMS](SHA256SUMS)。
- 包内包含采集器、OpenXR loader、C++ runtime、KTX 库和第三方许可证 assets。

工具链：JDK 17.0.20.1、Gradle 8.5、AGP 8.1.4、Build Tools 33.0.1、NDK 27.0.12077973、CMake 3.22.1、OpenXR loader 1.1.53；Meta OpenXR SDK 固定提交 `bbed2f20e38a5df7113630771c83cb8279e4fc26`。

2026-09-10 至 11 日从当时清理后的构建目录执行 `./gradlew --no-daemon --console=plain assembleDebug lintDebug` 成功。Lint 为 **0 errors、4 warnings**：固定旧 target API、未支持 ChromeOS x86、Android 12 备份规则提示、未设置 launcher icon。编译依赖另有旧 minizip 原型及 NDK 新增 ABI 元数据提示；交付 APK 的实际 ABI 已核对为 arm64-v8a。

2026-09-14 在 `bimanual-teleop/quest_app/` 用相同工具链和已校验 SHA-256 的固定 Meta OpenXR SDK 缓存，离线执行 `./gradlew --offline --no-daemon --console=plain assembleDebug -PmetaSdkSource=/home/yuchen/.cache/bimanual-teleop/meta-openxr-sdk-v85` 成功；`lintDebug` 为 0 errors、4 warnings。新 APK 已更新到交付目录，v2 签名验证通过，签名证书与旧 APK 相同，包内不再含原项目目录的绝对路径。更名前的交付 APK 保存在 `../../../demo-archive/20260914-folder-rename/quest-capture-debug-before-rename.apk`。新 APK 尚未在 Quest 上实机复验；以下实机数据均来自更名前的 APK。

本机复用工具链的命令（在 `quest_app` 内运行）：

```bash
JAVA_HOME=/home/yuchen/.cache/bimanual-teleop/jdk17 \
ANDROID_HOME=/home/yuchen/.cache/bimanual-teleop/android-sdk \
GRADLE_USER_HOME=/home/yuchen/.cache/bimanual-teleop/gradle-home \
./gradlew --no-daemon --console=plain assembleDebug lintDebug
```

工具安装在用户缓存目录，ADB 可从 `~/.local/bin/adb` 调用。首次 wrapper 下载遇到网络超时后，使用同一官方分发包且校验 SHA-256 填充缓存；最终构建通过项目 wrapper 完成。

## 自动检查

当时 Python 自动检查 **25 项通过**。此处是历史基线，不代表当前测试数量。

- 原生测试编译同一份采集 C++，用小型 XR 测试替身核对三次共同时间查询、LOCAL / grip / VIEW 空间、原点生效边界、独立追踪标志及刷新率请求失败；实际输出 JSON 再交给 Python 解码。
- Python 检查位置与非零旋转换基、valid / tracked 区分、部分失效、旧会话、损坏消息、序号缺口、原点事件缺失、生效顺序、USB EOF 和主机收流停顿。
- 连续样本逐帧发布；重复读取 latest 不新增样本或接收时间；源序号缺口明确报告。
- 包导入与类型检查无设备副作用。Python wheel 构建、临时目录安装和脱离源码导入均通过。
- 当时的 CLI `--help` 通过；无头显时明确报告 `ADB: no devices` 并返回退出码 1，无异常堆栈。

## 实机验收待办

Quest 3S 和 Quest 3 分别记录设备型号、Horizon OS 版本、运行配置及结果：

- [ ] 安装启动成功，头显及双手柄实际 90 Hz 输出，记录平均频率和时间间隔。
- [ ] 颈挂连续运行 30 分钟，静止不按键、翻腕、双手交叉、遮挡及恢复。
- [ ] 固定手柄后移动头显，确认 LOCAL 中手柄不产生头显相对运动。
- [ ] 系统菜单、休眠、重定位、USB 断开：状态变化、参考系版本和缺口可追溯。

Quest 3S 已完成下述佩戴状态一分钟试采；上述跨设备及完整场景验收仍未完成。手腕标定、机器人基座变换、跨设备时钟映射及机器人控制属于后续模块。

## Quest 3S 首次连接试采（2026-09-10）

设备 `340YC10GB00HL5`，Quest 3S，Android 14 / API 34，系统 build incremental `3296320034600610`。USB 协商速率 5000 Mbps，ADB 已授权；当时的交付 APK 安装、启动成功。

60 秒历史试采收到 5080 帧，源序号缺口 0。运行时报告 90 Hz；整段源帧平均频率 85.38 Hz，查询间隔中位数 11.11 ms，最大 1027.14 ms。期间记录两次 XR 会话停止、恢复以及六次 LOCAL 参考系变更；零序号缺口不代表采集没有停顿。

| 对象 | 位姿有效比例 | 完整追踪比例 |
| --- | ---: | ---: |
| 头显 | 100.00% | 92.48% |
| 左手柄 | 47.13% | 31.89% |
| 右手柄 | 42.93% | 31.85% |

三路均完整追踪且应用有焦点的帧占 24.88%；最长连续片段约 5.47 秒，该片段实际 90.00 Hz。用户确认本轮手柄尚未拿起或唤醒，头显放置状态未确认，因此本次仅确认设备连接、APK 运行和三路位姿接收，不作为正常握持或颈挂追踪稳定性验收。

原始日志 `data/quest-3s-check-20260910-01.jsonl`、统计与设备信息 `data/quest-3s-check-20260910-01-summary.json` 均在上述归档中。试采结束已关闭本次应用。

## Quest 3S 佩戴状态试采（2026-09-10）

用户确认戴好头显、拿起并唤醒双手柄后，使用同一设备和 APK 运行第二轮 60 秒试采。共接收 5357 帧，实际源帧平均频率 **89.983 Hz**，主机平均到达频率 89.986 Hz，运行时报告 90 Hz。源序号缺口 0，结束前设备健康状态为 ready。

| 对象 | 位姿有效比例 | 完整追踪比例 |
| --- | ---: | ---: |
| 头显 | 100.000% | 100.000% |
| 左手柄 | 99.981% | 99.589% |
| 右手柄 | 99.981% | 99.981% |

启动首帧双手柄尚未激活。左手柄在启动后约 17.60 秒和 50.17 秒处，分别有 13 帧和 8 帧的 `POSITION_TRACKED` 标志下降，至下一完整追踪帧约 144.28 ms 和 88.89 ms；这些帧位置及朝向仍 valid，不能因此标成完整追踪。原始标志均已保留，具体原因本轮未验证。

查询间隔中位数 11.11 ms、P95 11.29 ms、最大 27.33 ms；主机到达间隔 P95 40.72 ms、最大 42.24 ms，说明到达节奏存在波动。没有超过 100 ms 的查询或接收间隔，初始化后没有 XR 会话停止或 LOCAL 参考系变更。尚未完成时钟映射，不能由这些间隔推导 USB 单向延迟或端到端遥操作延迟。

本轮确认了正常佩戴、双手柄已唤醒时，Quest 3S 可通过此 APK 和 USB 链路输出约 90 Hz 的三路位姿。颈挂 30 分钟、固定手柄移动头显、物理轴向与左右对应、遮挡恢复专项、系统菜单/休眠/重定位/拔线以及 Quest 3 实机验收仍待执行。

原始日志 `data/quest-3s-check-20260910-02.jsonl`、统计与帧间隔 `data/quest-3s-check-20260910-02-summary.json` 均在上述归档中。已核对日志帧数与汇总一致，试采结束已关闭本次应用。

## 切换扩展坞后的连接检查（2026-09-10）

用户将 Quest USB 线从电脑直连接口移至扩展坞后，ADB 仍识别同一序列号 `340YC10GB00HL5`，状态为已授权的 `device`，USB 路径由 `4-1` 变为 `4-1.3`。Quest 链路协商速率仍为 5000 Mbps；其上游 Genesys Logic `05e3:0625` USB Hub 链路为 10000 Mbps，同一 Hub 还接有 Realtek `0bda:8153` 网卡。

随后尝试通过 ADB shell 读取型号、APK 路径及进程时，三条命令均返回 `device not found`，说明连接检查期间设备已离线；原因仍在检查，尚不能确认扩展坞连接稳定。此次未重新运行位姿采集；上节 60 秒结果属于直连配置，不能视为扩展坞持续采集或共享负载测试结果。

## 更换 USB 集线器后的连接检查（2026-09-11）

新集线器为 Realtek `0bda:0423`，USB 3 上行速率 5000 Mbps；Quest 3S `340YC10GB00HL5` 位于 `4-1.1`，协商速率同为 5000 Mbps，ADB 状态为已授权的 `device`，原 APK 仍已安装。

30 秒内每秒发起一次只读 ADB 型号查询，30 次全部成功。检查期间集线器设备号保持 90、Quest 设备号保持 92，内核没有记录这条 USB 链路的断开、重新枚举或错误。检查前的 Quest 插拔记录不计入本次稳定性窗口。

本轮仅确认短时 USB 连接稳定，未启动位姿接收，不代表 90 Hz 输出、长期稳定性或多设备共享负载已经通过。原始检查记录 `data/quest-3s-hub-usb-20260911-01.json` 位于上述归档。

## 双手柄 3D 可视化（2026-09-11）

独立可视化入口 `scripts/view_quest.py` 使用 Matplotlib 3.10.9 / NumPy 1.26.4 显示接收端已经转换好的 LOCAL 位姿。后台实时读帧与 GUI 定时刷新分开；查看器不逐帧记录数据。

当时 Quest 模块 28 项测试通过，其中 6 项覆盖查看器的四元数轴方向、单侧失效、有效但未追踪、过期/失焦数据隐藏、视野调整及关闭窗口释放设备。

使用历史 Quest 3S 第 1000 帧渲染并检查布局，预览图 `data/quest-view-preview.png` 位于上述归档。随后在当前 USB 集线器上实际启动查看器，GUI 收到样本的更新共 619 次，观察到源序号从 0 增至 2188，其中 493 次更新同时显示左右手柄。未观察到源错误或收流结束；窗口关闭后程序退出码 0，并释放本次 Quest 会话。该检查验证实时绘图和生命周期，不代替连续追踪或端到端延迟验收。
