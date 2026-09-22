# 左臂外编码器清零后持续无法进入位置模式（2026-09-21）

用户报告：在认为是零点的姿态清零外编码器并重新上电后，左臂持续报错误 4。截图显示 Status=100、ArmError=4，Position 为 [79.957, -83.086, 7.061, -15.948, -7.909, 3.671, 9.724]°。机械臂实体姿态是否变化尚待用户确认，不能仅凭读数变化认定实际运动或原点丢失。

## 本次已核实的证据

2026-09-21 08:33:57 UTC，使用随包 SDK 独立 RecvFile / TCP 10240 文件通道下载日志及 robot.ini。没有机器人控制连接、清错、使能、运动、松闸或参数修改。

- 日志明确记录 RESETEXTENC0：轴 0 两次、轴 1 一次、轴 2 一次，对应左 J1/J2/J3。相应操作前日志有下伺服进入 IDLE 的记录；不能据此验证当时的实体机械零位。
- 每次清零后约 130–143 个日志周期，对应轴报 0xFF35。固定 SDK 字典与随包 JMDT 手册 PDF 第 15 页（纸面第 13 页）均将其定义为“驱动器过流 2”。日志仅证明报警码及时间关系，不能据此认定实际短路、硬件损坏，或将其当作清零后的正常提示。
- 随后的模式请求反复出现 Link0 或 Link1 的内外编码器检查错误。最新 POSITION 请求在控制器显示时间 00:29:06、周期 408789，下一周期 408790 报 Link1，即左 J2。不能继续只排查 J1。
- 本次 robot.ini 与 06:07 UTC 下载的文件逐字节相同。这不能证明外编码器零点未变，因为尚未确认该操作是否将数据写入驱动器或其他存储；回写旧 robot.ini 没有已验证的恢复依据。
- 用户提供的厂商说明明确要求机械零点位置及下使能/复位状态。此前上位机 Position 为零只能证明该反馈通道的显示零位，不能独立验证机械零位。上位机源码清零按钮只发送所选轴的 RESETEXTENC0，并未验证机械零位，也未检查该调用的返回值。

## 下一步与限制

先停止反复清零、清错使能、松闸及回零尝试。在当前稳定姿态下只读保存 Position 与 PositionEx，以及当前各关节伺服错误；记录断电前后实体姿态有无变化。应由厂商结合机械零位标记、编码器及驱动器参数确认恢复/标定流程，并核实本机驱动器版本下 0xFF35 的含义。不得从截图差值推算并写入补偿，不应通过扩大 EncErrorValve 绕过检查。

用户尚未回答具体操作经过及实体姿态变化。本次未改变项目控制代码，也没有做实机运动验证。

## 原始日志摘录

时间是控制器显示时间，未与主机校准；行号对应本次下载的完整日志。

```text
48148: [INFO][22:41:28              5645212][PSI]OnProcessParam: Set RESETEXTENC0 0
48149: [INFO][22:41:28              5645213][PSI]OnProcessParam: Reset Arm0.Link0's external encoder offset
48150: [ERRO][22:41:28              5645343][SI]_FX_RunIdle: arm0: Servo error{0xff35,0x00,0x00,0x00,0x00,0x00,0x00} happened, transfer to ARM_STATE_ERROR

48207: [INFO][22:42:31              5707661][PSI]OnProcessParam: Set RESETEXTENC0 0
48208: [INFO][22:42:31              5707661][PSI]OnProcessParam: Reset Arm0.Link0's external encoder offset
48209: [ERRO][22:42:31              5707791][SI]_FX_RunIdle: arm0: Servo error{0xff35,0x00,0x00,0x00,0x00,0x00,0x00} happened, transfer to ARM_STATE_ERROR

48231: [INFO][22:43:11              5747958][PSI]OnProcessParam: Set RESETEXTENC0 1
48232: [INFO][22:43:11              5747958][PSI]OnProcessParam: Reset Arm0.Link1's external encoder offset
48233: [ERRO][22:43:11              5748101][SI]_FX_RunIdle: arm0: Servo error{0x00,0xff35,0x00,0x00,0x00,0x00,0x00} happened, transfer to ARM_STATE_ERROR

48253: [INFO][22:43:36              5773422][PSI]OnProcessParam: Set RESETEXTENC0 2
48254: [INFO][22:43:36              5773422][PSI]OnProcessParam: Reset Arm0.Link2's external encoder offset
48255: [ERRO][22:43:36              5773552][SI]_FX_RunIdle: arm0: Servo error{0x00,0x00,0xff35,0x00,0x00,0x00,0x00} happened, transfer to ARM_STATE_ERROR

50623: [INFO][00:28:53               395147][PSI]Arm0: Request a transfer to POSITION
50624: [INFO][00:28:53               395147][SI]_FX_UpdateCmd: arm0 receive a request to transfer to POSITION
50625: [INFO][00:28:53               395147][SI]_FX_RunIdle: arm0 transfer to ARM_STATE_TRANS_TO_POSITION
50626: [ERRO][00:28:53               395148][SI]__FX_RequestTransToPositionMode: arm0: Link1's encoder/extencoder might be error.
50627: [ERRO][00:28:53               395148][SI]_FX_RunTransferToPosition: arm0: Transfer to ARM_STATE_ERROR

50643: [INFO][00:29:06               408789][PSI]Arm0: Request a transfer to POSITION
50644: [INFO][00:29:06               408789][SI]_FX_UpdateCmd: arm0 receive a request to transfer to POSITION
50645: [INFO][00:29:06               408789][SI]_FX_RunIdle: arm0 transfer to ARM_STATE_TRANS_TO_POSITION
50646: [ERRO][00:29:06               408790][SI]__FX_RequestTransToPositionMode: arm0: Link1's encoder/extencoder might be error.
50647: [ERRO][00:29:06               408790][SI]_FX_RunTransferToPosition: arm0: Transfer to ARM_STATE_ERROR
```

完整文件保存在 `/tmp/tianji_after_encoder_zero_20260921T083357Z/`。SHA-256：

- controller_LOG.txt：`4e7eb4166988713d3c93ad3a19366487112d1384da787f69e937f84d0a6df9b0`
- robot.ini：`0e13870676aa69fbb585ae161094d4a3583644f6d2a874923d332eb74374c027`
