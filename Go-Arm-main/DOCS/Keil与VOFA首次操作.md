# Keil、VOFA 和 Event Recorder 首次操作

适用于当前 `Go_Arm_Final/Go-Arm-main/Go-Arm-main` 工程。代码已修改并做编译检查，以下板上步骤尚未替你操作。详细变量说明见 [机械臂调试入门](机械臂调试入门.md)。

## 1. 先分清两个连接

| 电脑软件 | 连接 | 用途 |
|---|---|---|
| Keil | ST-LINK/J-Link 等调试器 → 板上 SWD 接口 | 下载、看 Watch、看 Event Recorder |
| VOFA+ | USB-TTL → 板上 UART9 | 接收六通道波形 |

USB-TTL 接线：板子 PD15/TX → 转换器 RX，板子 GND → 转换器 GND，电平需兼容3.3V。只接收波形不需要连接转换器 TX。不要把转换器的供电脚随意接到已经供电的板上。
PD15 是芯片引脚名，实际排针位置以你们的板卡原理图为准。调试器的 USB 接线不自动等于 UART9 接通；带虚拟串口的调试器也需要正确连到 UART9。
保持机构禁用，先验证通信。首次进入调试或下载可能复位芯片；不要在机构运动中这样操作。

## 2. 代码里的开关：已经替你处理

`Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 中现为：

```c
VOFASendTaskHandle = osThreadNew(VOFA_SendTask, NULL, &VOFASendTask_attributes);
```

这行不能有 `//`。此前确实遗漏了这一处，发送函数存在但任务没创建。
`Config/app_config.h` 中 `APP_VOFA_ENABLE=1`，周期为10 ms。
真正发送的函数在 `Application/src/myostasks.c`；`freertos.c` 中同名的 `__weak` 空循环是备用实现，不需要把它再改成发送代码。
编译成功只能证明程序能构建；运行时还要看任务句柄和发送计数。

## 3. Keil：打开、编译、连接、下载

1. 打开本工程 `MDK-ARM/27RC_Proj_Template.uvprojx`。核对路径，避免打开旧的 Go-Arm-main。
2. Project → Build Target（F7）。底部 Build Output 应显示0个错误。
3. Project → Options for Target（Alt+F7）→ Debug。
4. 选择右侧实际硬件调试器：ST-LINK 或 J-LINK 等，不能选 Use Simulator 来看实机反馈。已能正常下载的配置优先保留。
5. 点 Debug 页对应的 Settings，检查是否识别调试器和目标芯片，SWD 连接是否正常。这里检测不到芯片时先排查供电、SWD接线和调试器驱动，不需要改 VOFA 代码。
6. Utilities 页使用对应的下载驱动，Flash Download 设置保留你们已经验证的 H723 配置，不照搬别的 STM32 型号算法。
7. 确认机构安全、禁用后，Flash → Download。查看输出是否下载/校验成功。当前工程配置允许进入 Debug 前更新 Flash，因此进入调试也可能触发下载。
8. Debug → Start/Stop Debug Session（Ctrl+F5）进入调试。界面切换后，Debug → Run（F5）。进入调试并不一定已经运行。

## 4. Keil Watch：先证明程序真的跑起来

View → Watch Windows → Watch 1，在空白 Name 行双击输入变量，每项按 Enter：

| 输入项 | Run 后的预期 |
|---|---|
| `VOFASendTaskHandle` | 非0，表示任务创建成功 |
| `vofa_sent_count` | 持续增加，表示持续成功启动发送 |
| `vofa_skipped_count` | 发送未结束时跳过的次数 |
| `vofa_start_error_count` | HAL 启动DMA失败的次数，理想为0 |
| `Unitree_link[0].rx_count` | 电机0有效回包计数；电机通信运行时应增加 |
| `Unitree_link[1].rx_count` | 电机1有效回包计数 |
| `g_arm_diag.event_init_ok` | 初始化后为1 |

如运行时窗口不刷新，检查 View → Periodic Window Update。硬件调试驱动需支持运行中读写内存；不要通过反复暂停运动中的CPU刷新变量。
数字是十六进制时可在右键菜单调整显示方式，是否持续变化比显示进制更重要。
`vofa_sent_count` 增长只证明启动DMA，不证明电脑收到；USB-TTL拔掉时它仍可能增长。

## 5. VOFA+：先接收，再绑定曲线

1. 将 USB-TTL 插入电脑。Windows 设备管理器 → 端口(COM和LPT)，记下新增的 COM 编号。没有端口先处理转换器驱动/USB连接。
2. 打开 VOFA+，选择串口数据源，端口选上一步的 COM。
3. 设置115200波特率、8数据位、1停止位、无校验；流控如有选项，选无。
4. 协议选 **JustFloat**，不要照官方快速开始示例选 FireWater；本工程发送的是二进制浮点帧。
5. 打开串口/开始接收。确保其他串口助手没有占用同一个COM。
6. 从控件栏拖出波形图，在波形图右键菜单中选择绑定Y轴，先勾通道0和1。
7. 点击 Auto 自动调整Y轴范围，把时间位置滑块移到最新端。静止时两条水平线也是正常数据，不一定必须起伏。
8. 再加一个波形图绑定2和3；最后一张绑定4和5。不要将rad和Nm强行使用同一量程。
9. 在波形的采样间隔/Δt设置中填10 ms；如果单位是秒，填0.01。这只是按固定间隔换算显示，任务抖动/丢帧会影响精确时间轴。

默认0/1是电机0目标/反馈角，2/3是电机1目标/反馈角，4/5是两只电机的反馈力矩。
先确认静止数据与 Watch 大致一致，再按队内既有操作执行已验证的小动作。无需为观察曲线随意修改目标或使能变量。

## 6. Event Recorder：先用手动事件验证，无需动电机

本工程已包含 EventRecorder.c 和初始化，不需要再在 RTE 勾选第二份库，不需要接 SWO 或额外配置 printf。

1. 在机构禁用时退出 Debug，进入 Options for Target → Debug。
2. 打开 **Manage Component Viewer Description Files**。
3. 点 **Add Component Viewer Description File**，添加 `MDK-ARM/ArmDiagnostics.scvd` 并确认。这一步用于将编号翻译成文字。
4. 重新进入 Debug 并 Run。
5. View → Analysis Windows → Event Recorder。若窗口有 Enable/Enable Recorder，确保已启用；显示过滤器包含 Arm / 0x30 组件及 Op、Error 级别。
6. 在 Watch 添加 `g_diag_capture_request`，双击其 Value 列，把0改成1并按Enter。保持程序Run。
7. 机械臂任务处理后它会变回0；事件窗口应出现 **ManualSnapshot**，时间列显示记录时间。
8. 如果只看到 `0x300C`，先检查SCVD是否添加成功；这个编号就是手动快照，并不表示故障。
9. 添加 `g_arm_diag.log_count`、`g_arm_diag.log`。展开结构体，看到同一次记录的目标、反馈、年龄等。这是RAM快照；事件窗口不会自动展开所有结构体成员。

这一步通过后，再观察正常动作的 TrajectoryStart 和 CommandsSent_ArrivalUnverified。后者仅表示指令发完、到位尚未验证，不表示判定到位失败。旧显示名称 CommandsSent_NOT_Arrived 容易误解，现已修正；事件ID和控制逻辑不变。
事件不是每10 ms刷一条，静止且没有异常/请求时没有新行是正常现象。

## 7. 看不到数据时按顺序查

| 现象 | 下一步 |
|---|---|
| Watch显示找不到变量 | 确认工程路径、下载的是新构建、处于Debug；不要先动优化选项 |
| `VOFASendTaskHandle=0` | 确认已Run过初始化；检查任务创建语句和FreeRTOS堆是否足够 |
| 句柄非0但发送计数不变 | 检查APP_VOFA_ENABLE、CPU是否Run、错误计数和跳过计数 |
| 发送只增长一次，跳过不断增长 | 检查DMA/UART发送完成中断和回调，不要强行循环清忙标志 |
| 发送计数增长，VOFA无通道 | 查COM、PD15到RX、共地、波特率、JustFloat、串口占用 |
| 有通道但看不到线 | 检查Y轴绑定、Auto、是否显示最新时间段 |
| 电机角度不变 | 检查回包计数和反馈年龄；静止与通信断开都可能表现为角度不变 |
| Event窗口无ManualSnapshot | 先看capture_request是否归零、log_count是否增加、event_init_ok是否为1，再查窗口Enable/过滤器/调试器支持 |
| RAM记录有更新但事件窗口仍空 | MCU记录逻辑与PC读取是两段链路，继续查Keil版本/调试驱动/窗口配置 |

若必须暂停CPU或退出调试排查，先用已有可靠方式让机构安全停止。切勿把Keil的Stop/Halt当电机急停。

## 官方界面参考

### 导出事件：保存按钮变灰的准确原因

本机 Keil `UV4/uv4.chm` 的 Event Recorder Window 帮助明确要求：Halt debugging to enable the button。仅取消窗口 Enable 不足以启用保存按钮；取消 Enable 只停止调试器读取事件，MCU 中的事件记录代码仍可运行。

- 保存已有事件：先以可靠方式停稳机构，确保允许暂停CPU；保持Debug会话，使用 Debug → Stop 暂停CPU，再点击 Event Recorder 的软盘按钮，保存CSV。不要退出Debug、复位或点Clear。
- 后续连续记录：保持窗口Enable，在 View → Command Window 输入 `ER > C:\Users\17685\arm_events_20260926.csv`，用新文件名避免覆盖；输入 `ER` 查看日志状态；结束时输入 `ER OFF` 关闭日志文件。此方式适合后续事件持续落盘，已有历史优先用保存按钮导出。
- `ER SAVE` 保存的是过滤器配置，不是事件日志。

### 两个时间变量为什么可能显示相等

`Unitree_link[i].last_valid_rx_ms` 仅在收到有效回包时赋值；`g_arm_diag.latest.time_ms` 在诊断任务取得快照时赋值。两者都取HAL毫秒时钟，但触发条件和存储位置不同。Watch逐项读取，接收更新可能发生在两项读取之间，因此界面上这两个数不保证与保存的age_ms组成同一时刻的等式。稳定回包下age_ms长期保持1或2毫秒可以是正常的采样相位现象。若要逐帧回包间隔，需要另行增加接收端统计，当前未实现。

- [Arm：事件窗口与SCVD配置](https://arm-software.github.io/CMSIS-View/latest/er_use.html)
- [Keil：Watch窗口](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_watchwin.asp)
- [VOFA：拖出波形、绑定Y轴、Auto](https://www.vofa.plus/docs/learning/start/quick_start/)，其中FireWater示例不适用于本工程，应选JustFloat。
- [VOFA：波形与采样间隔](https://www.vofa.plus/docs/learning/widgets/wave/)
