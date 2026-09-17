# Quest OpenXR 客户端

采集头显与左右 Touch 手柄 grip 位姿，通过 USB ADB 传给主机。支持 Quest 3 / 3S，请求 90 Hz，头显内显示空白 XR 场景。

## 运行

先完成[项目安装](../README.md#安装)，在项目根目录分别运行：

```bash
conda activate bimanual-teleop
python scripts/view_quest.py
python scripts/teleop_quest_tianji.py --arms-only
```

查看器关闭窗口退出。遥操作启动步骤、参考系和左右对应关系见[操作指南](../README.md#遥操作与回位)。多个 USB 设备连接时，使用 `--serial SERIAL` 选择头显。

## 连接与追踪

1. 开启头显开发者模式，连接 USB 并授权 ADB；`adb devices -l` 应显示 `device`。
2. 唤醒头显和双手柄，保持手柄在头显摄像头可见范围内，周围环境应能正常追踪。
3. 关闭系统菜单、边界确认或权限窗口，返回采集应用；确认查看器报告有效追踪后再遥操作。

主机程序会请求亮屏并保持采集期间活跃，退出后恢复系统休眠检测。应用可在手柄未唤醒时先启动等待。Quest 3S 真正休眠后的传感器锁可能需要短按实体电源键解除；亮屏不表示 XR 已恢复焦点。休眠、追踪丢失或重新定位后，遥操作须重新接合。

主机异常结束或 USB 中断后，如应用残留或休眠检测未恢复，可执行：

```bash
adb shell am force-stop org.bimanual.questcapture
adb shell am broadcast -a com.oculus.vrpowermanager.automation_disable
```

嵌入其他程序时，`QuestSource(keep_awake=False)` 可保留系统原有电源行为。

## 构建

设置 JDK 17 的 `JAVA_HOME` 和 Android SDK 的 `ANDROID_HOME`，在 `quest_app/` 目录执行：

```bash
sdkmanager 'platform-tools' 'platforms;android-32' 'build-tools;33.0.1' 'ndk;27.0.12077973' 'cmake;3.22.1'
./gradlew assembleDebug
adb install -r build/outputs/apk/debug/QuestCapture-debug.apk
```

项目固定 Gradle 8.5、Android Gradle Plugin 8.1.4、OpenXR loader 1.1.53，目标为 arm64-v8a，最低 Android API 26。交付 APK 位于 `artifacts/quest-capture-debug.apk`。

首次构建下载并校验 Meta OpenXR SDK 提交 `bbed2f20e38a5df7113630771c83cb8279e4fc26`。可用 `-PmetaSdkSource=/path/to/meta-openxr-sdk-v85` 指向同一提交的本地源码。

## 独立检查采集

在 `quest_app/` 目录执行，将 `SERIAL` 替换为设备序列号：

```bash
adb -s SERIAL logcat -v raw -T 0 QuestCapture:I '*:S'
```

另开终端启动应用，每次生成新会话 ID：

```bash
SESSION_ID=$(tr -d '-' < /proc/sys/kernel/random/uuid)
adb -s SERIAL shell am force-stop org.bimanual.questcapture
adb -s SERIAL shell am start -n org.bimanual.questcapture/.MainActivity --es session_id "$SESSION_ID"
```

直接 ADB 启动不包含主机程序的电源管理。每行输出为 JSON；Python 接收器按会话 ID 过滤。原始坐标、追踪标志和消息格式见[协议参考](../docs/development.md#quest-协议)。

检查双手柄位置和旋转、遮挡、重新定位、系统菜单及 USB 断开。源码协议测试包含在根目录的 Python 测试中；构建通过仍需在目标头显上检查追踪行为。
