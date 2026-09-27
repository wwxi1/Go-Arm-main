# Event Recorder详细信息与导出

现在异常事件后会附带关键现场数据，沿用原来Event Recorder窗口的CSV保存流程。正常运动及XYArrivalSuccess仍只显示简短事件，不持续输出详细快照。VOFA六通道不变。

## 下次调试操作

1. 在机构停稳、允许重新启动的条件下，F7编译并下载新固件，再进入Debug运行。
2. 使用原来的 `MDK-ARM/ArmDiagnostics.scvd`，无需再添加一个。文件已更新，重新进入Debug让Keil加载新定义。若新事件只有编号，检查原SCVD路径和勾选状态。
3. 最简单的验证无需运动：未使能时，在Watch将 `g_diag_capture_request` 写1。看到ManualSnapshot和后续7条Snapshot详细行，说明新代码开始记录了。
4. 日常只看 `g_arm_comm_ok` 和事件窗口。故障发生时自动生成详细行，不需要每次手动请求。
5. 导出时先可靠停稳机构、确保允许暂停CPU，再Debug → Stop暂停调试，点击事件窗口软盘保存CSV。仅取消窗口Enable不足以启用保存按钮。保持需要的事件可见，查看全部类型，别只筛选红色Error，否则可能看不到Op级别的详细行。

CSV仍是原来的Event、Time、Component、Event Property、Value列；新数据在Snapshot行的Value中。它不会自动变成一张“每个变量一列”的波形表。不要先Clear、复位或重新下载再保存。

## 会多出哪些数据

| 触发事件 | 后续详细行 |
|---|---|
| FeedbackTimeout / FeedbackRecovered | 上下文，以及对应电机的反馈年龄、驱动错误码 |
| DriveError | 上下文，以及对应电机的链路、目标/反馈角、速度、力矩 |
| XYArrivalTimeout | 上下文、两台电机的数据、实际用于检验的目标XY及反馈估算XY |
| InvalidTarget / ManualSnapshot | 上下文和两台电机的数据 |
| 正常开始、发完指令、成功到位 | 不追加详细行 |

每组先出现原来的主事件，接着SnapshotContext和若干详细行。同一组详细行的 `seq` 相同，等于RAM主事件的sequence。它不是Keil左侧Event列的行号；重新初始化后从新一轮计数开始。

| 详细行 | 字段含义 |
|---|---|
| SnapshotContext | seq、所属动作state、采样时间sample_ms、通信超时阈值timeout_ms |
| SnapshotLink | 电机编号motor、反馈年龄age_ms、驱动错误码drive_error |
| SnapshotAngles | target_mrad和feedback_mrad；除以1000得到rad |
| SnapshotLoad | speed_mrad_s除以1000得到rad/s；torque_mNm除以1000得到N·m |
| SnapshotXYGoal | 检验目标x_mm_x10/y_mm_x10和容差tolerance_mm_x10；除以10得到mm |
| SnapshotXYFeedback | 反馈角度经正运动学估算的XY，单位同上；valid=1才是本次检验可用的反馈 |

例如 `feedback_mrad=-600` 就是-0.600 rad；`x_mm_x10=-3859` 就是-385.9 mm。MCU只存定点整数，不调用printf拼字符串；字段名称由电脑上的SCVD解释。浮点值非有限或缩放溢出时用 `-2147483648` 表示无效；age_ms为4294967295表示从未收到有效反馈。

`valid=0` 时，XY可能只是旧反馈换算出的坐标，不能当成当前位置。50 mm故意偏移测试时，SnapshotXYGoal显示偏移后的检验位置，电机实际目标不变。

InvalidTarget后的SnapshotAngles是当时电机正在使用的指令，不是被拒绝的输入XY。当前并未导出所有非法输入参数。快照也没有持续保存故障前后的完整波形，需要观察运动过程时仍使用VOFA。

## 开销和边界

没有新增Flash写入、串口发送、动态内存分配或阻塞等待，也没有扩大事件缓存。详细记录只在机械臂任务的事件路径中生成，不放进电机接收中断或定时器中断。取得快照时短暂关中断；后续换算与Event Recorder调用都在恢复原中断状态后进行。

工程的Event Recorder缓冲仍为128个16字节槽。EventRecord2占1槽，EventRecord4占2槽。下面是一次主事件加详细行的占用，不是额外申请的静态RAM：

| 一次事件 | 窗口行数 | 缓冲槽数 / 字节 |
|---|---:|---:|
| 普通事件/成功到位 | 1 | 1 / 16 |
| 单电机超时或恢复 | 3 | 5 / 80 |
| 单电机驱动报错 | 5 | 9 / 144 |
| 手动快照或目标非法 | 8 | 15 / 240 |
| 到位超时 | 10 | 19 / 304 |

持续断线只记录一次，恢复时再记录一次；反复断开恢复仍会产生多组记录。缓冲有限，调试器未及时读取时旧数据仍可能覆盖，不能当成无限历史。`g_arm_diag.event_dropped`只累计API写入失败，不统计覆盖掉的旧记录。RAM的16条队列只保存主事件及完整快照，附加行不挤占这16条。

## 本次代码检查与优化

- 异常判定、RAM保存、Event Recorder详细行共用触发判断时的快照，避免判定后新回包令日志与故障条件不一致。
- 发完指令时复用一次采样结果，减少重复采样。
- 将XYArrivalTimeout纳入首次故障快照。
- 删除未调用的Arm_Position_Process、未使用的ARM_MOVE_SKY_TIME和旧注释代码块。
- 诊断模块、相关运动入口、宇树驱动和VOFA发送中的英文说明改成中文；第三方库和许可证不改。
- 保持通信阈值、到位容差、运动控制和告警不自动停机的行为。

官方依据：[EventRecord2/4的数据记录接口](https://arm-software.github.io/CMSIS-View/main/group__EventRecorder__Data.html)、[记录槽占用](https://arm-software.github.io/CMSIS-View/1.2.0/er_theory.html)。板上事件格式和CSV导出仍需下载后确认；本地API测试不能替代探针和Keil界面的验证。

## 已完成验证

本地测试同时编译运行Event Recorder关闭/开启两种配置。检查了详细行与主事件序号关联、负数和无效浮点编码、写入失败计数、同一快照复用、正常到位不追加详细行、50 mm偏移目标导出及失效反馈标记；原有通信、到位、测试请求、RAM队列和VOFA发送测试仍通过。

SCVD通过本机Keil的Component_Viewer.xsd校验，27个事件编号无重复。Keil编译为0错误，仍有原有的RAM_D3未匹配段警告；当前ZI-data=30024字节，与增加详细行之前相同，RW-data仍为868字节。未在本次工作中下载或操纵电机；尚未实测板上执行耗时。
