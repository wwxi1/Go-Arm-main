# Go-Arm 工程代码复查报告

- **复查对象**：`D:\RC_Study_project\Go-Arm-main\Go-Arm-main`
- **对照版本**：旧版 `D:\RC_Study_project\WHU_STM32H7_Template-main (1)\WHU_STM32H7_Template-main`
- **日期**：2026-09-16
- **说明**：本报告只做分析与定位，**未修改任何工程文件**。

---

## 0. 结论速览

| 等级 | 条目 | 位置 |
| --- | --- | --- |
| **P0** | 宇树**反馈帧结构体实际 18 字节**，协议是 16 字节 → CRC 永不通过 → **整条反馈通路是死的** | `UnitreeMotor.h:79-85, 60-69` |
| **P0** | 力矩**编码把减速比乘反**（应除），误差 GR² ≈ 40 倍，且 int16 溢出（潜伏，当前未触发） | `UnitreeMotor.c:188` |
| P1 | `DJ_Current` 模式**限幅完全无效**（返回值被丢弃） | `DJmotor.c:381` |
| P1 | `DJmotor_Monitor` 被注释 → **堵转/超时保护失效** | `DJmotor.c:360` |
| P1 | "首拍对齐"（bumpless transfer）是**死代码**，实际被关掉 | `UnitreeMotor.c:36, 518-522` |
| P1 | 速度/位置**限幅常数忽略减速比**（速度真实上限 127 而非 804） | `UnitreeMotor.c:184-185` |
| P1 | `Is_pick` 与 `Is_place` 同时为 1 → **命令被静默丢弃** | `seize_sky.c:445, 485, 530` |
| P1 | `PulsePerRound = 8191`（应为 8192） | `DJmotor.c:58, 64` |
| P1 | 笛卡尔坐标入口 / 夹爪是**半成品空壳** | `seize_sky.c:360`、`holding_jaw.c/h` |
| P1 | **零点只在开机瞬间捕获**，复位前必须回标定位（操作约束） | `UnitreeMotor.c:137, 533-537` |
| P2 | 注释与代码符号相反（`(u1-u2)` vs `(u1+u2)`），按注释改会把 bug 改回来 | `kinematics.c:12`、`kinematics.h:14` |
| P2 | 宇树下发**丢帧仍推进 round-robin 索引** | `UnitreeMotor.c:540-546` |
| P2 | 死代码：`zero_pos`、`Target_Vec`、`Quintic_Traj` | 多处 |

**已修复（相比旧版，见第 2 节）**：正逆运动学符号不自洽（旧版 P0）已解决；标定值已换成实测值。

---

## 1. 版本变化总览

| 模块 | 旧版 | 新版 Go-Arm |
| --- | --- | --- |
| 目录结构 | 扁平（`User/ Motor/ Application/ Config/`） | 分层（新增 `BSP/ Communication/ DOCS/`，`Config/` 集中） |
| 宇树驱动 | 单缓冲 + 手写逐字节解析 | **重写**：UART7 + RS485（`HAL_RS485Ex_Init`）+ DMA 双缓冲 + 环形缓冲 + IDLE 中断 + CRC16 |
| 宇树反馈零位 | 无 | 新增 `zero_offset`，收发双向补偿 + 开机 `SetZero` |
| 运动学标定值 | Z1=2.44, Z2=−2.75, R=5/6, L1=L2=400 | **Z1=0.0, Z2=2.3562(135°), R=2/3, L1=389.5, L2=423.42**（实测标定） |
| `U2_Motor2Geom` | `(u1+u2)*R + Z2`（与逆解矛盾） | `Z2 − (u1+u2)*R`（**与逆解自洽**） |
| 调试入口 | 无 | 新增 `ARM_DEBUG` 状态 + `Arm_Debug_Process()` + `Arm_Debug` 结构体 |
| 运动学自检 | 未定义（符号搜不到） | `Arm_Kinetics_Data_t` 落地在 `seize_sky.h:155-163`，1ms 任务里刷新 |
| 夹爪 | 无 | `holding_jaw.c/h` 空壳（`main.c`、`IQRhandler.c` 已 include） |
| VESC | 无 | 新增 `VescMotor.c`（`USE_VESC 0`，未启用） |

---

## 2. 已修复 / 已确认无问题的部分

### 2.1 【旧版 P0 已修复】正逆运动学符号自洽

旧版 `U2_Motor2Geom` 用 `(u1 + u2)`，而注释、头文件、逆解三处都是 `(u1 − u2)`，导致 `Forward(Inverse(p)) ≠ p`。

新版两侧改成互逆：

```c
// kinematics.c:28-31
float U2_Motor2Geom(float u1, float u2)
{
    return ARM_U2_ZERO_POS - (u1 + u2) * ARM_U2_RATIO;
}
// kinematics.c:39-43
static float U2_Geom2Motor(float theta1, float theta2)
{
    float u1 = U1_Geom2Motor(theta1);
    return (ARM_U2_ZERO_POS - theta2) / ARM_U2_RATIO - u1;
}
```

数学验证：`θ2 = Z2 − (u1+u2)R` ⟺ `(Z2−θ2)/R = u1+u2` ⟺ `u2 = (Z2−θ2)/R − u1`。**严格互逆。**

**数值验证**（取 `ARM_U1_HIGH_POS / ARM_U2_HIGH_POS`，Z1=0, Z2=2.3562, R=2/3, L1=389.5, L2=423.42）：

| 环节 | 计算 | 结果 |
| --- | --- | --- |
| 输入 | u1 = 1.33, u2 = 1.18 | — |
| θ1 | u1 + 0 | 1.330 rad = 76.2° |
| θ2 | 2.3562 − (2.51)(2/3) | 0.6829 rad = 39.13° |
| Forward | x = 389.5cos(76.2°) + 423.42cos(115.3°) | **−88.3 mm** |
| Forward | y = 389.5sin(76.2°) + 423.42sin(115.3°) | **761.0 mm** |
| Inverse r² | 7810 + 579121 | 586931 |
| cosθ2 | (586931 − 151710 − 179284)/(2·389.5·423.42) = 255937/329845 | 0.77593 → **θ2 = 39.11°** ✓ |
| β | atan2(761.0, −88.3) | 96.6° |
| α | atan2(423.42·sin39.11°, 389.5 + 423.42·cos39.11°) | 20.4° |
| θ1 | β − α | **76.2°** ✓ |
| u1' | 76.2° − 0 | **1.330** ✓ |
| u2' | (2.3562 − 0.6829)/(2/3) − 1.33 = 2.5100 − 1.33 | **1.180** ✓ |

**自检闭环 `inverse_angle == forward_angle` 现在精确成立。** 你写的 `Arm_Kinetics_Data` 那段调试代码终于能对上了。

### 2.2 八个点位宏的 θ2 全部落在 [0°, 180°]（反证标定是对的）

| 状态 | u1 | u2 | θ2 = 2.3562 − (u1+u2)·(2/3) |
| --- | --- | --- | --- |
| START | 0 | 0 | 135.0° |
| READY | 1.6 | 0.35 | 60.5° |
| SKY_READY / LOW | 1.643 | −0.15 | 78.0° |
| KEEP | 1.16 | 0.72 | 63.2° |
| SKY | 1.71 | −0.5 | 88.8° |
| LOW1 | 1.6 | 0.55 | 52.9° |
| MID | 1.31 | −0.02 | 85.7° |
| MID1 | 1.25 | 0.60 | 64.3° |
| HIGH | 1.33 | 1.18 | 39.1° |

`Inverse` 的 `acosf` 值域是 `[0, π]`。**所有点位都落在值域内**，说明 `Z2` 和 `RATIO` 这两个标定值选对了——如果标定错了，至少会有点位算出反向解、自检闭环对不上。

### 2.3 【上次结论需撤回】"跨上下文竞态"不成立

全部外设中断抢占优先级都是 **5**（`tim.c:205` TIM2、`fdcan.c:206-272` FDCAN1-3、`usart.c:458` UART7、`dma.c:48-75`）：

```c
HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
```

Cortex-M 中断**同优先级互不抢占**，所以：

- `Arm_State_Update()`（TIM2）与 `Arm_Receive()`（FDCAN3）天然互斥 ✓
- `UnitreeMotor_Func()`（TIM2）与 `UnitreeMotor_UART_RxHandler()`（UART7）天然互斥 → `Unitree_motors[i].data = *rx_data` 这个结构体整体赋值不会被撕裂 ✓

`IQRhandler.c:105-107` 的注释是对的。我上次基于旧版结构提的那条竞态可以划掉。

> 代价：所有中断串行执行。TIM2 的 handler 里串了 `Arm_State_Update + 3×CAN_DequeueTx + UnitreeMotor_Func + DJmotor_Func`，会阻塞 UART7/FDCAN 的响应。1kHz（1ms 预算）目前够用，但以后 FDCAN 流量上来了要重新评估。
>
> 另：`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`（`FreeRTOSConfig.h:144`），优先级 5 的 ISR 恰好**可以**调 `FromISR` API（合法边界）。当前这些 ISR 都没调，没问题；若以后要调，注意别再往低编号调。

---

## 3. P0 问题

### 3.1 【P0】宇树反馈帧结构体尺寸错 → 反馈通路是死的

**现象推导链**：

```c
// UnitreeMotor.h:60-69
typedef struct
{
    int16_t  torque;            // 2
    int16_t  speed;             // 2
    int32_t  pos;               // 4
    int8_t   temp;              // 1
    uint8_t  error : 3;         // ← 声明成 uint8_t
    uint16_t foot_force : 12;   // ← 换成 uint16_t，起新的存储单元
    uint8_t  none : 1;          // ← 又换回 uint8_t，再起一个新单元
} UnitreeMotorWireFbk_t;
```

C 语言规则：**位域声明类型改变时会另起存储单元**（即使剩余位够用）。所以：

- `error` 占 1 字节（只用 3 bit）
- `foot_force` 另起 2 字节
- `none` 再另起 1 字节

→ `WireFbk_t = 13` 字节 → `UnitreeMotorFeedbackFrame_t = 2 + 1 + 13 + 2 = 18` 字节。

**实测验证**（用 GCC 复现同一份结构体定义）：

```
sizeof WireMode=1  WireCmd=12  WireFbk=13  Ctrl=17  Fbk=18
fbk bytes (13 B): 22 11 44 33 88 77 66 55 60 05 BC 0A 01
                                              ↑  ↑  ↑   ↑
                                            temp err force none
```

**但协议要求的是 16 字节**（`head[2] + mode[1] + torque[2] + speed[2] + pos[4] + temp[1] + status[2] + crc[2]`），工程自己也是按 16 字节设计的：

```c
// UnitreeMotor.h:30
#define UNITREE_FRAME_LENGTH 16U          // ← RX 解析器攒 16 字节就解析
// UnitreeMotor.c:251-253
data->calc_crc = crc_ccitt(0, packet, sizeof(*frame) - sizeof(frame->crc16));  // 应该是 14
rx_crc = (uint16_t)packet[14] | ((uint16_t)packet[15] << 8U);                  // CRC 在 [14..15]
```

`sizeof(*frame)` 实际是 **18**，于是：

| 环节 | 代码 | 实际后果 |
| --- | --- | --- |
| `crc_ccitt(0, packet, 18-2=16)` | 应覆盖 14 字节 | **把 CRC 自己也算进去了**，覆盖 [0..15] |
| `memcpy(&rx_data->frame, s_unitree_frame, sizeof(rx_data->frame))` | 拷 18 字节 | 从 16 字节数组**越界读 2 字节垃圾**（`UnitreeMotor.c:329`） |
| `rx_crc = packet[14] \| packet[15]<<8` | 读 [14..15] | 这两个字节在计算范围内 → **永远对不上** |

**后果**：`data->correct` 永远为 0（或偶发 1/65536 的巧合）。于是——

1. `Unitree_motors[i].data.position` **恒为 0**（只在 `correct==1` 分支里才写入）
2. `UnitreeMotor_SetZero()` **永不执行**（`UnitreeMotor.c:533` 要求 `data.correct`）→ `zero_offset` 恒为 0
3. `Arm_Interpolation_Start()` 读到 `start_angle = data.position = 0` → **每段轨迹的起点恒为 0，而不是臂的当前位置**

第 3 条最危险：你上次问过"任务没跑完就切换模式会怎样"，答案是"从当前位置平滑接过去"。**现在这个保护失效了**——中途切状态时，新轨迹的 `start_angle = 0`，第一个 1ms 周期就会把 `cmd.position` 从"当前实际值"瞬间写成 `0 + (target−0)·s(0.001)` ≈ 0。**电机被命令值瞬间拉回零位，会产生一次猛烈抽动。**

**修法**（已用 GCC 验证）：三个位域统一用 `uint16_t` 声明，强制共用一个 2 字节单元：

```c
typedef struct
{
    int16_t  torque;
    int16_t  speed;
    int32_t  pos;
    int8_t   temp;
    uint16_t error : 3;        // uint8_t  → uint16_t
    uint16_t foot_force : 12;  // 已经是 uint16_t
    uint16_t none : 1;         // uint8_t  → uint16_t
} UnitreeMotorWireFbk_t;
```

验证输出：

```
fix: WireFbk=11  FbkFrame=16  (目标 11 / 16)
temp@整帧[11]=60   status@整帧[12..13]=D5E5
  error      = status & 0x7      = 5     (期望 5)      ✓
  foot_force = (status>>3)&0xFFF = 0xABC (期望 ABC)    ✓
CRC 覆盖长度 = sizeof(frame) - 2 = 14                  ✓
```

位域位置和 CRC 覆盖长度**全部与 `DecodeFrame` 里的 `packet[]` 下标对上**了。

**并建议加编译期断言，防止以后再被静默改坏**：

```c
/* UnitreeMotor.h 里，#pragma pack() 之后 */
_Static_assert(sizeof(UnitreeMotorControlFrame_t)  == 17U, "unitree ctrl frame must be 17B");
_Static_assert(sizeof(UnitreeMotorFeedbackFrame_t) == 16U, "unitree fbk frame must be 16B");
```

> ⚠️ 我用的是 GCC 复现。你的工程是 Keil MDK + AC6（armclang），位域分配规则**可能**与 GCC 有差异。上面两行断言正好是"在你自己编译器上一句话验证"的办法——加进去编译，不报错就说明布局对了。如果 AC6 下原来就是 16 字节，那 `correct` 能通过，本节可以忽略，但断言仍建议保留。
>
> 快速判据：在调试器里看 `Unitree_motors[0].data.correct`。**如果是恒 0，本节成立**；如果恒 1，说明 AC6 的布局和 GCC 不同。

### 3.2 【P0·潜伏】力矩编码把减速比乘反了

```c
// 解码 UnitreeMotor.c:276
data->torque = ((float)torque) / 256.0f * UNITREE_GEAR_RATIO;   // joint = wire/256 × GR
// 编码 UnitreeMotor.c:188
tor_des = (int16_t)(cmd->torque * UNITREE_GEAR_RATIO * 256.0f); // wire = joint × GR × 256  ← 反了
```

编码应是解码的逆运算，即 `wire = joint / GR × 256`。现在两边都**乘** GR → 差 **GR² = 6.33² ≈ 40 倍**。

对比另两路（这两路是**对的**）：

| 物理量 | 编码 | 解码 | 互逆？ |
| --- | --- | --- | --- |
| 位置 | `pos × GR/2π × 32768` | `2π·pos/32768 / GR` | ✓ |
| 速度 | `spd × GR/2π × 256` | `spd/256 × 2π / GR` | ✓ |
| **力矩** | `tau × GR × 256` | `tau/256 × GR` | ✗ **差 GR²** |

**附带溢出**：`UNITREE_SATURATE(cmd->torque, -127.99f, 127.99f)` 之后乘 1620.5 → `tor_des` 可达 **±207,000**，远超 `int16_t`（±32767），`(int16_t)` 强转会**回绕成任意值**。改成 `÷GR` 后最大值是 `127.99/6.33×256 = 5177`，正好落在 int16 内——这也反证 `÷GR` 才是本意。

**当前为什么没炸**：`Arm_Motor_Enable()` 只设 `MODE_FOC`（位置环，用 kp/kd），`cmd.torque` 在 `Init` 里被 `memset` 成 0，从没被写过。**一旦你用前馈力矩、或切 `UNITREE_MOTOR_MODE_*` 的力矩模式，会瞬间给到 40 倍力矩。**

**修法**：`tor_des = (int16_t)(cmd->torque / UNITREE_GEAR_RATIO * 256.0f);`

---

## 4. P1 问题

### 4.1 `DJ_Current` 模式限幅完全无效

```c
// DJmotor.c:379-382
case DJ_Current:
    /* 直通电流:任务层每周期写 valSet.current_raw,这里补限幅 */
    ClampPeak(DJmotor[i].valSet.current_raw, DJmotor[i].param.CurrentLimit_raw);
    break;
```

`ClampPeak` 是 `static inline float`，**返回限幅后的值**（`mathFunc.h:103-111`），这里返回值被丢弃。

对比同一个文件里正确的用法（`DJmotor.c:257, 280`）：

```c
motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
```

**修法**：加上赋值。否则任何走 `DJ_Current` 的代码都能把任意大的电流推进 C620。

### 4.2 堵转 / 超时保护被注释掉

```c
// DJmotor.c:358-360
if (DJmotor[i].Begin)
{
    // DJmotor_Monitor(&DJmotor[i]);   ← 整段屏蔽
```

`DJmotor_Monitor` 里的两个保护（堵转 → 失能、50 次没收到反馈 → 失能）**全部失效**。CAN 掉线时电机保持最后一条电流命令。建议至少放开超时分支，或在任务层加心跳计数。

### 4.3 "首拍对齐"是死代码

```c
// UnitreeMotor.c:36
static bool set_cur_init = true;          // ← 初值就是 true
// UnitreeMotor.c:518-522
if (!set_cur_init && motor->data.correct) // ← 条件恒为假，永不进入
{
    motor->cmd.position = motor->data.position;
    set_cur_init = true;
}
```

`set_cur_init` 是 `static`，全文件只被赋 `true`，**没有任何地方置 `false`** → 这段永不执行。旧版是能跑的首拍对齐，新版等于**关掉了**。

后果：使能瞬间 `cmd.position` 是残留值。目前靠开机 `SetZero` 把 `cmd.position` 归 0 掩盖了，但**如果你在失能状态下手动掰动机械臂再使能，电机会直接弹回残留目标位**。

**修法**：把初值改成 `false`（让它真的在首帧反馈时对齐一次），或者删掉这段——别留一段"看起来在防护、实际是空的"代码。

### 4.4 速度/位置的限幅常数忽略了减速比

```c
// UnitreeMotor.c:184-185
UNITREE_SATURATE(cmd->speed,    -804.00f, 804.00f);
UNITREE_SATURATE(cmd->position, -411774.0f, 411774.0f);
```

- `804.00 = 32767 × 2π / 256`
- `411774 = 2π × 2³¹ / 32768`

两个都是**不带 GR** 时的满量程。但编码里乘了 GR，所以实际安全上限是：

| 量 | 常数上限 | 带 GR 后的真实上限 | 现在超限会怎样 |
| --- | --- | --- | --- |
| speed | 804 rad/s | **804 / 6.33 ≈ 127 rad/s** | `>127` 时 `(int16_t)` 回绕成反向大速度 |
| position | 411774 rad | 65050 rad | 永远够用，无害但常数不对 |

你的轨迹速度量级是 1~3 rad/s，离 127 很远，属**理论隐患**；但常数和公式不匹配说明这层没统一推导过。修法：`±(32767 × 2π / (256 × GR))` 和 `±(2π × 2³¹ / (32768 × GR))`。

### 4.5 `Is_pick` 与 `Is_place` 同时为 1 → 静默丢命令

```c
// seize_sky.c:416 与 456
if (Is_pick==1 && Is_place==0) { ... return; }
if (Is_place==1 && Is_pick==0) { ... return; }
// 两个都进不去 → 落到 seize_sky.c:530
Is_pick=0; Is_place=0; Is_store=0; Is_ready=0; Is_reset=0; Is_keep=0;
```

两个都置 1 时两个 `if` 都失败，最后统一清零 → **这一条指令被静默吞掉，状态机不动**。不会卡死，但会"丢命令"，且没有任何错误标记。

**修法**：二选一 —— 加一个 `Is_pick && Is_place` 的显式分支（记 `Is_ok` 报错 / 规定 pick 优先），或至少置一个错误位便于现场排查。

### 4.6 `PulsePerRound = 8191`（应为 8192）

```c
// DJmotor.c:58, 64
dj2006_param.PulsePerRound = 8191U;
dj3508_param.PulsePerRound = 8191U;
```

C620/C610 一圈是 **8192** 计数。用 8191 会让 `angle_deg` 有约 0.012% 的增益误差，**长行程下累计漂移**（转 100 圈就差 12 圈计数量）。`DJmotor_AngleCalculate` 的回卷判据 `ABS(PulseGap) > 4096` 是按半圈写的，改成 8192 后判据仍然成立（半圈 = 4096）。

### 4.7 笛卡尔坐标入口与夹爪是半成品

| 位置 | 状态 |
| --- | --- |
| `seize_sky.c:127-137` `Arm_Interpolation_Pos_Start()` | 整段注释掉（给 `(x,y)` → `Inverse` → `Start` 的路径） |
| `seize_sky.c:360-368` `Arm_Position_Process()` | **定义了但没有被任何 `switch` 调用**，且函数体照抄 `Arm_KEEP_Process` 的 KEEP 宏 |
| `seize_sky.c:31` `volatile Vec2 Target_Vec;` | 声明后全工程未使用 |
| `holding_jaw.c/h` | 空壳（只有 `#include`），但 `main.c:45`、`IQRhandler.c:31` 已 include |

这三处指向同一件事：**"末端笛卡尔坐标 → 逆解 → 下发" 这条链还没打通**。要做的话：

1. 先修 3.1（否则 `data.position`/`SetZero` 都是死的，标定无从谈起）
2. 再验证 `Forward(Inverse(p)) == p`（现在数学上成立了）
3. 然后注意：**在 `Start` 里解一次逆解只能保证"终点正确"**，中途路径仍是关节空间的弧线。想要末端走直线，必须"每 1ms 在笛卡尔空间插补 + 每周期调一次 `Inverse`"

夹爪/气缸可以直接搬旧工程 `Ground_Block` 里那套 `solenoid_init(3)` + `solenoid_on(3, 位图)` 位-bang 电磁阀驱动（新版 `BSP/bsp_solenoid.c` 有对应位置）。

### 4.8 零点只在开机瞬间捕获（操作约束）

```c
// UnitreeMotor.c:137   Init 里设一次
Unitree_motors[i].set_zero = true;
// UnitreeMotor.c:533-537  Func 里第一次收到有效反馈就消费掉，之后再也不设
if (motor->set_zero && motor->data.correct) { UnitreeMotor_SetZero((int)i); motor->set_zero = false; }
```

`SetZero` 做的是 `zero_offset += data.position; cmd.position = 0; data.position = 0;` —— **把"开机那一刻机械臂所在的姿态"定义成 0**。

- GO-M8010-6 是绝对编码器，`data.position` 每次上电是确定值 → **只要每次开机姿态一致，就是可复现的**
- 但如果你上次把机械臂留在半空，下次开机零点就整体偏了 → `ARM_U1_LOW_POS = 1.643` 等**全部点位整体平移同一个量**，而且是静默的

你代码里的 `Is_Sys_reset`（`0x010202F0` 收到 `'R'` → `NVIC_SystemReset()`）正好就是这套机制：**"校准"= 把臂摆到标定位再复位系统**。设计上自洽，但**必须写进操作手册**，否则现场极易出现"点位全错但没人知道为什么"。

另外 `zero_pos` 字段（`UnitreeMotor.h:129`，注释"认为的设定零点以后的几何角度"）在 `Init` 里被赋了 `ARM_U1_ZERO_POS / ARM_U2_ZERO_POS`，然后**全工程再没被读过**。于是"零点"概念分裂成三处：

| 位置 | 是否在用 |
| --- | --- |
| `kinematics.h` 的 `ARM_U*_ZERO_POS` | ✓ 用于几何角换算 |
| `UnitreeMotor.h` 的 `zero_pos` | ✗ **死字段** |
| 运行时 `zero_offset` | ✓ 真正生效 |

**建议**：删掉 `zero_pos`，或者让它真正承担"固定的机械参考角"职责——那样就不需要每次开机同姿态了（绝对编码器可以直接把 `zero_offset` 硬编成标定姿态的绝对读数）。

---

## 5. P2 问题

### 5.1 注释与代码符号相反

```c
// kinematics.c:12（文件头注释）
 *   小臂几何角 theta2 = (u1 - u2) * ARM_U2_RATIO + ARM_U2_ZERO_POS
// kinematics.h:14
// 小臂角度减速比: 小臂几何角 = (大臂电机角 - 小臂电机角) * ARM_U2_RATIO
```

代码现在是 `Z2 − (u1+u2)·R`，注释还写着 `(u1−u2)·R + Z2`。**方向（差/和）和符号（+/−）两处都不一样。** 下次有人"照注释改代码"就会把刚修好的自洽性重新破坏。请同步改注释。

### 5.2 宇树下发丢帧仍推进索引

```c
// UnitreeMotor.c:419-423
if (uart->gState != HAL_UART_STATE_READY) { return; }   // 静默丢弃
...
// UnitreeMotor.c:540-546
UnitreeMotor_SendCommand(&Unitree_motors[s_unitree_tx_index]);
s_unitree_tx_index++;                                     // 无论成功与否都前进
```

RS485 半双工，上一帧还在发时这一帧被丢掉，但轮询索引照常前进 → 该电机这一轮就不发了。标称 500Hz（2 电机 / 1kHz），丢帧会让它变成不规则。

**修法**：让 `UnitreeMotor_SendCommand` 返回 `bool`，只在成功发送后才 `s_unitree_tx_index++`。

### 5.3 死代码 / 重复实现

| 符号 | 位置 | 说明 |
| --- | --- | --- |
| `set_cur_init` | `UnitreeMotor.c:36` | 见 4.3 |
| `zero_pos` | `UnitreeMotor.h:129` | 见 4.8 |
| `Target_Vec` | `seize_sky.c:31` | 声明后未使用 |
| `Arm_Position_Process` | `seize_sky.c:360` | `static` 且无调用者 |
| `Quintic_Traj` | `mathFunc.c:130` | **无人调用**；且固定 `10t³−15t⁴+6t⁵`，正是 `Target_Quintic_Interpolation` 在 `v0=acc0=0` 时的特例 —— **两份五次多项式实现** |
| `holding_jaw.c/h` | `User/src,inc` | 空壳 |

建议：`Quintic_Traj` 要么删，要么改成接受 `v0/acc0` 参数后由 `Target_Quintic_Interpolation` 调用它，只保留一份。

### 5.4 一批可调参数需要核对

| 参数 | 当前值 | 位置 | 备注 |
| --- | --- | --- | --- |
| DJ 位置环 KP | 0.07 | `DJmotor.c:125` | 位置环输出直接串到速度环作 `SetVal`，KP 太小 → 位置环带宽极低、跟踪滞后大。若真要用 `DJ_Position` 控大关节，建议 0.3~1.0 起步扫参 |
| DJ 速度环 KI | 0.25 | `DJmotor.c:126` | 增量式 PID，Kp=5.5/Ki=0.25 配比可用，但注意增量式的**二次积分**特性 |
| 宇树 KP / KD | 4.8 / 0.024 | `motor_config.h:70-71` | `kd = 0.024` 相对 `kp = 4.8` 看起来**阻尼严重不足**（多数 GO 例程用 kp≈20、kd≈0.5~1）。若观察到到点抖动/振荡，先加 kd |
| 宇树 KP/KD 宏命名 | `MOTOR_UNITREE_DEFAULT_KW` | `motor_config.h:71` | 赋值给 `cmd.kd`，但宏名是 `KW`（刚度 Kw 的写法？）。建议统一成 `KD` 免混淆 |

### 5.5 `ARM_INTERPOLATION_DT` 硬编码

`seize_sky.c:7` 写死 `0.001f`，配合 `osDelay(1)` + `configTICK_RATE_HZ = 1000`（`FreeRTOSConfig.h:67`）。

**当前无漂移**：我们上次估算每轮 work ≈ 50µs ≪ 1ms 的 tick 周期，`osDelay(1)` 会自己重新对齐 tick 边界，误差不累积。新版每毫秒多了 `Forward()` + `Inverse()`（含 `acosf` + 两次 `atan2f`），M7 带硬件 FPU，仍是 µs 量级。

属**防御性**建议：将来往任务里加更重的活（视觉、复杂规划），或把 `osDelay` 改成 `osDelay(2)`、移植到 tick ≠ 1ms 的平台，`time += 0.001` 的假设就会静默失效。

### 5.6 段间速度不连续（换挡会"跳"）

```c
// seize_sky.c:561-566
if (ArmControl.state != ArmControl.last_state)
{
    ArmControl.running = false;
    ArmControl.finish  = false;
    ArmControl.last_state = ArmControl.state;
}
```

state 一变就把 `running` 清零 → 新轨迹从**当前实际位置**起跑（不瞬移 ✓，前提是 3.1 修好），但 `ArmControl.time` 归零、`s'(0) = 0.8` 重新起跳 → **段间速度是"跳"的，不是"接"的**。

五次多项式本身 `s'(1) = 0`，所以每段都**稳稳停在终点**，下一段再从 0.8 冲起来 —— 速度曲线呈锯齿状。要做连续过渡才需要之前聊过的 blend（`(1−α)·posA + α·posB`）。

> 顺带修正我上一轮的说法：`v0=0.8, acc0=4.6` 实算系数是 `a1=0.8, a2=2.3, a3=−1.7, a4=−1.7, a5=1.3`，峰值 `s' = 1.595 @ t=0.33`、峰值 `|s''| = 4.6 @ t=0`。这两个峰值都**低于**全归零版（`1.875` / `5.77`）。所以它不是"起跳冲击大"，真实特征是"**起点有速度/加速度阶跃 + 节奏偏前**"。风险随 `Δθ/T²` 放大：`SKY→KEEP` 段 `Δdj = 65°` 才会有 `299°/s²` 的加速度阶跃。

### 5.7 `level_flag == 0` 分支撞状态 + 两个点位宏完全相同

```c
// Is_ready && level_flag==0  → ARM_STATE_SKY_READY   (seize_sky.c:394)
// Is_place && level_flag==0  → ARM_STATE_SKY_READY   (seize_sky.c:492)
```

"天空块取块准备"和"天空块放块"落到**同一个状态、同一个点位**。而 `ARM_U1_LOW_POS / ARM_U2_LOW_POS / ARM_DJ_LOW_POS`（1.643 / −0.15 / −222.0）与 `ARM_U1_SKY_READY_POS / ...` **三个值完全相同**。

请确认：是**有意的位姿复用**，还是复制粘贴漏改？`Is_keep` 同样不响应 `level_flag`（无论 `level_flag` 怎么变，KEEP 都是同一姿态）。

### 5.8 `Arm_Kinetics_Data` 没有输出通路

```c
// seize_sky.c:554-560（在 Arm_Control_Task 里，1ms 刷新）
Arm_Kinetics_Data.motor_target = Arm_Debug;
Arm_Kinetics_Data.arm_angle.Angle1 = U1_Motor2Geom(Unitree_motors[0].data.position);
Arm_Kinetics_Data.arm_angle.Angle2 = U2_Motor2Geom(Unitree_motors[0].data.position, Unitree_motors[1].data.position);
Arm_Kinetics_Data.forward_angle.u1_theta = Unitree_motors[0].data.position;
Arm_Kinetics_Data.forward_angle.u2_theta = Unitree_motors[1].data.position;
Arm_Kinetics_Data.end_coordinate = Forward(Arm_Kinetics_Data.forward_angle);
Arm_Kinetics_Data.inverse_angle = Inverse(Arm_Kinetics_Data.end_coordinate);
```

这段逻辑是对的（正解↔逆解互检，正是验证 3.1 / 2.1 的正确做法），但**只能在调试器里看**：`VOFA_SendTask` 目前只发 `Unitree_motors[0].data.position` 一路通道（`myostasks.c:44`）。

想让它在 VOFA 上看出"闭环对不对"，需要加通道，例如：

```c
VOFA_Channel_Update(0, VOFA_TYPE_FLOAT, (void *)&Arm_Kinetics_Data.forward_angle.u1_theta);
VOFA_Channel_Update(1, VOFA_TYPE_FLOAT, (void *)&Arm_Kinetics_Data.inverse_angle.u1_theta);
VOFA_Channel_Update(2, VOFA_TYPE_FLOAT, (void *)&Arm_Kinetics_Data.end_coordinate.x);
VOFA_Channel_Update(3, VOFA_TYPE_FLOAT, (void *)&Arm_Kinetics_Data.end_coordinate.y);
```

两条 `u1_theta` 曲线重合 = 正逆解自洽；`Angle2` 应该等于 `RAD2DEG(θ2)`。

---

## 6. 建议的动手顺序

1. **加两行 `_Static_assert`** 到 `UnitreeMotor.h`，编译一遍 → 立刻知道 3.1 在你编译器上是否成立（成本最低、信息量最大）
2. 修 3.1 的位域类型（若断言报错）；修 3.2 的 `tor_des` 用 `÷GR`
3. 在调试器里确认 `Unitree_motors[0].data.correct` 变为 1、`data.position` 随手动掰动而变化 → **这时才说明反馈通了**
4. 通了之后，用 `Arm_Kinetics_Data` 的自检闭环验证标定（`inverse_angle == forward_angle`）
5. 修 4.1 / 4.2 / 4.5（三处都是"写着但没生效"的类型）
6. 4.6 / 4.3 / 4.4 收尾
7. 再做 4.7（笛卡尔入口 + 夹爪）

---

## 附：本轮用到的验证脚本（可复现）

结构体布局用 GCC 复现过，脚本在 `D:\Claw file\2026-08-05-17-22-14\tmp_unitree_layout.c` 和 `tmp_unitree_fix.c`，编译命令：

```bash
gcc -O0 -o out.exe tmp_unitree_layout.c && ./out.exe
```

输出（原版）：

```
sizeof WireMode=1 WireCmd=12 WireFbk=13 Ctrl=17 Fbk=18
```

输出（位域改成 uint16_t 后）：

```
fix: WireFbk=11  FbkFrame=16  (目标 11 / 16)
```
