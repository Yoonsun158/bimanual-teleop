# 双臂机器人遥操作

Quest 手柄控制天机机械臂，Wuji Glove 控制 Hand2。控制命令只有添加 `--enable-motion` 才会运动；省略时只读或预览。

## 准备

以下命令在项目根目录执行，Python 统一使用 Conda 环境 `bimanual-teleop`：

```bash
# 首次创建环境
PIP_USER=false conda env create -f environment.yml
conda activate bimanual-teleop
```

主机需有 ADB、CMake 和 C++17 编译器。Quest 开启开发者模式，连接 USB，并在头显内授权 ADB：

```bash
adb devices -l
adb install -r quest_app/artifacts/quest-capture-debug.apk

# 首次使用天机机械臂前构建桥接库；此步骤需要联网获取 SDK
cmake -S tianji_bridge -B tianji_bridge/build -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"
cmake --build tianji_bridge/build -j4
```

Quest 查看和遥操作命令会启动采集应用。佩戴头显并保持应用在前台；连接多个 Quest 时，用 `--serial SERIAL` 选择设备。

手动关闭头显上正在运行的采集应用（Quest Capture）：

```bash
adb shell am force-stop org.bimanual.questcapture
```

连接多个 ADB 设备时，使用 `adb -s SERIAL shell am force-stop org.bimanual.questcapture`，将 `SERIAL` 替换为 `adb devices -l` 中对应头显的序列号。

## 配置

- `configs/tianji_teleop.json`：填写 `controller_ip`；`profile` 是机械臂运动参数，`ready_pose` 是回位目标，`quest.coordinate_frame` 选择手柄参考系。
- `configs/wuji_teleop.json`：填写左右 `devices` 地址和已有标定用户名 `sdk_user_name`；空用户名使用 SDK 默认用户。查看和单独手部控制可用 `--user-name NAME` 临时覆盖。

参数含义和数组顺序见配置内注释。配置支持 `//` 和 `/* */` 注释，不支持尾逗号；修改后重启程序生效。

```bash
# 天机入口默认读取 configs/tianji_teleop.json；仅自定义路径时加 --tianji-config PATH
# Wuji 入口默认读取 configs/wuji_teleop.json；仅自定义路径时加 --wuji-config PATH
# 联合遥操作同时读取上述两份配置
```

## 查看

```bash
# Quest 手柄位姿
python scripts/view_quest.py

# 左手套骨架、接触位置与压力；右手改为 --side right
python scripts/view_wuji_glove.py --side left

# 右臂六维力，默认读取 10 帧；加 --count 0 持续读取，Ctrl+C 退出
python scripts/read_tianji_right_force.py
```

手套颜色表示相对压力，黑框表示 SDK 检测到接触，压力值不是牛顿。`CONTACT UNKNOWN` 表示缺少有效接触模型或接触流；此时仅显示压力。触觉来自手套，Hand2 Beta2 没有触觉反馈。

六维力输出单位为 N、N·m。读取另需天机 Python SDK，默认位于项目旁的 `../TJ_FX_ROBOT_CONTRL_SDK`，可用 `--sdk-root PATH` 指定。运行前先退出其他天机程序，避免占用同一连接。

## 控制与回位

```bash
# 双臂双手联合遥操作，默认读取天机和 Wuji 两份配置
python scripts/teleop_quest_tianji.py --enable-motion

# 只用 Quest 控制双臂；单臂再加 --side left 或 --side right
python scripts/teleop_quest_tianji.py --arms-only --enable-motion --side left

# 手套控制左侧 Hand2；右侧改为 right，双手改为 both
python scripts/teleop_wuji_hand2.py --enable-motion --side left

# 键盘点动左臂：启动先回初始位姿，到位后按 Enter 开始点动
python scripts/jog_tianji.py --enable-motion --side left

# 天机双臂回配置 ready_pose；只回一侧时加 --side left 或 --side right
python scripts/home_tianji.py --enable-motion

# 左侧 Hand2 回到 20 个关节零角，默认 3 秒到位并保持，Q 退出
python scripts/home_wuji_hand2.py --enable-motion
```

双手回零使用 `--side both`：先左手，确认到位并去使能后再回右手，全部完成后退出；中止或失败时不继续下一侧：

```bash
python scripts/home_wuji_hand2.py --enable-motion --side both 
```

Quest 遥操作启用运动后也会先将所选机械臂移到初始位姿，再等待接合。遥操作和点动的准备阶段可用 Space、Q 或 Ctrl+C 中止；单独运行天机回位命令时用 Ctrl+C。

| 操作 | 按键或手势 |
| --- | --- |
| 遥操作、键盘点动开始／恢复 | Enter |
| 暂停／取消等待 | Space |
| 退出 | Q |
| 联合模式开始／恢复 | 双手同时比 V 保持 0.3 秒，也可用 Enter |
| 联合模式暂停 | 任一手摇滚手势保持 0.3 秒，也可用 Space |
| 键盘点动平移 | W/S、A/D、R/F：原生基座 X/Y/Z 正负方向，默认每键 5 mm |
| 键盘点动旋转 | I/K、J/L、U/O：绕原生基座 X/Y/Z 正负方向，默认每键 2° |

遥操作设备未就绪时按 Enter，会等待就绪后自动接合，Space 可取消。追踪、反馈或 IK 异常会暂停，恢复后需重新接合。同一设备一次只运行一个入口；需要详细状态时加 `--verbose`。

### Quest 参考系与左右对应

左手柄控制右臂，右手柄控制左臂。`--side` 表示机器人侧，例如 `--arms-only --side left` 使用右手柄控制左臂；Wuji 仍是左手套控制左 Hand2、右手套控制右 Hand2。

- `"headset"`（默认）：原点跟随头显位置，水平朝向只跟随头显 yaw，竖直轴始终向上，忽略头显俯仰和侧倾。
- `"world"`：使用固定的 Quest LOCAL 世界坐标系。

所选参考系的前、左、上映射到机器人的相同物理方向，接合握姿不改变平移方向。位置和旋转均相对接合时的位姿计算增量；头显模式下，仅移动头显或改变其 yaw 也会改变手柄的相对位姿。

## 手套标定

左右手分别执行，按终端提示完成动作。把命令中的 `yuchen` 换成你的用户名；首次标定会创建用户，后续使用同名用户更新模型，完成后将用户名写入配置的 `sdk_user_name`。

```bash
# 使用 configs/wuji_teleop.json 中对应侧的手套地址
python scripts/calibrate_wuji_glove.py --kind joints --side left --user-name yuchen
python scripts/calibrate_wuji_glove.py --kind tactile --side left --user-name yuchen
```

动作说明见官方[关节标定图示](https://docs.wuji.tech/docs/en/wuji-studio/latest/calibration/)和[触觉标定图示](https://docs.wuji.tech/docs/en/wuji-studio/latest/tactile-calibration/)。触觉标定全程保持无接触，目前要求 24×31 传感器数据；触觉含义见 [SDK 文档](https://docs.wuji.tech/docs/en/wuji-glove/latest/sdk-data-reference/tactile/)。

## 测试

```bash
conda run -n bimanual-teleop python -m unittest discover -s tests -q
ctest --test-dir tianji_bridge/build --output-on-failure
```

实机按只读查看、单侧小幅运动、暂停与断流、双侧联合的顺序验收。
