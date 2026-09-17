# 武汉大学机器人队工程C语言规范

> 适用范围:基于 STM32 HAL 库的嵌入式 C 语言开发。
> 本规范的目标是保证基本的工程秩序:命名可读、作用域可控、模块边界清晰、代码可维护。遵循本规范,应能避免"全局变量乱飞""命名靠猜""一个文件写到底"等常见问题。

## 1. 总则

1. **一致性优先**:同一个工程内,同类符号必须使用同一种风格,禁止同一语义出现多种写法。
2. **可读性优先于简洁**:宁可用稍长的名称换取明确含义,不用只有作者才懂的缩写。
3. **模块化**:每个外设、每个功能模块独立成对文件(`.c` / `.h`),一个文件只做一件事。
4. **最小暴露**:能 `static` 就不全局,能不 `extern` 就不暴露,对外接口越少越好。
5. **与库保持一致**:与 HAL 库交互的代码,命名风格尽量贴近 HAL,降低阅读切换成本。

## 2. 文件与模块

### 2.1 文件命名

- 全部小写,单词之间用下划线 `_` 分隔,文件名体现模块职能。
- 头文件与源文件同名成对出现。

```
motor_control.c / motor_control.h
can_bus.c       / can_bus.h
pid_controller.c / pid_controller.h
```

### 2.2 头文件保护

所有头文件必须使用 `#ifndef` 保护,宏名用"模块名大写 + `_H`":

```c
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* 内容 */

#endif /* MOTOR_CONTROL_H */
```

## 3. 命名规范

### 3.1 通用原则

- 名字应表达"是什么",而不是"放在哪"。`motor_speed` 优于 `ms`,更优于 `tmp1`。
- 变量、函数用小写单词;类型、枚举、宏、常量按规定大小写(见下文)。
- 同一语义在工程内使用同一单词:速度统一用 `speed`,不用 `velocity` 和 `vel` 混用。

### 3.2 变量命名

- 采用 **小写蛇形命名**(snake_case):全小写,单词之间用下划线。

```c
uint16_t motor_speed;
float    pid_output;
uint8_t  uart_rx_data;
```

- **指针**加 `p_` 前缀,指向同一对象的指针保持一致:

```c
uint8_t *p_rx_buffer;
Motor_t *p_motor;
```

- **静态文件级变量**(仅在单个 `.c` 内可见)加 `s_` 前缀:

```c
static uint8_t s_rx_index;
static uint32_t s_last_tick;
```

- **全局变量**必须加 `g_` 前缀,并紧跟模块名,使其作用域从名字上即可分辨:

```c
uint16_t g_motor_current_speed;   /* 属于 motor 模块 */
uint8_t  g_can_rx_count;          /* 属于 can 模块 */
```

### 3.3 函数命名

- 采用 **`模块名_动作`** 风格,单词首字母大写,贴近 HAL 库习惯:

```c
void Motor_Init(void);
void Motor_SetSpeed(uint16_t speed);
uint16_t Motor_GetSpeed(void);
void Can_SendFrame(CanFrame_t *p_frame);
```

- 动词放在末尾或紧跟模块名之后,语义为"对模块做什么":

```c
void    Led_On(void);
void    Led_Off(void);
uint8_t Uart_ReadByte(void);
```

- 回调函数、中断服务函数加 `_Callback` / `_IRQHandler` 后缀,便于识别:

```c
void Uart1_RxCallback(uint8_t data);
void TIM6_IRQHandler(void);
```

### 3.4 类型命名(typedef)

- `typedef` 定义的类型名用 **`模块名_名称` + `_t` 后缀**,单词首字母大写:

```c
typedef struct {
    float kp;
    float ki;
    float kd;
} Pid_Param_t;

typedef struct {
    uint16_t speed;
    uint8_t  direction;
} Motor_State_t;
```

### 3.5 枚举

- 枚举类型名按类型命名规则,以 `_t` 结尾。
- 枚举成员**全大写 + 下划线**,并以模块名作为前缀,避免不同模块的枚举值冲突:

```c
typedef enum {
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_ERROR,
} Motor_State_t;

typedef enum {
    CHASSIS_MODE_NORMAL = 0,
    CHASSIS_MODE_GYRO,
    CHASSIS_MODE_LOCK,
} Chassis_Mode_t;
```

- 有明确数值含义的枚举应显式赋值;无特殊要求的从 0 开始递增。

### 3.6 宏定义

- 常量宏、配置宏**全大写 + 下划线**,带模块前缀:

```c
#define MOTOR_MAX_SPEED        3000
#define UART_RX_BUF_SIZE       128
#define PID_SAMPLE_PERIOD_MS   5
```

- 宏名应表达含义,禁止无意义的魔法数字直接散落在代码里。
- 函数式宏同样全大写;仅用于确实无法用 `inline` 函数替代、且无副作用风险的场景(见第 6 节)。

### 3.7 缩写风格

- 只使用工程内公认、统一的缩写,首次出现的生僻缩写需在注释中给出全称。
- 常用缩写表(团队内统一,不要另造):

| 缩写 | 全称 | 缩写 | 全称 |
| --- | --- | --- | --- |
| `init` | initialize | `cfg` | config |
| `buf` | buffer | `idx` | index |
| `cnt` | count | `num` | number |
| `tmp` | temp | `curr` | current |
| `prev` | previous | `next` | next |
| `max` | maximum | `min` | minimum |
| `rx` | receive | `tx` | transmit |
| `adc` | analog-to-digital | `pwm` | pulse-width modulation |
| `pid` | proportional-integral-derivative | `msg` | message |

- 专业缩写(如 `adc`、`pwm`、`can`、`spi`、`i2c`、`uart`)保持小写,不再解释。
- 禁止随意截断单词,如 `speed` 不得写成 `spd`、`count` 不得写成 `ct`。

## 4. 作用域与全局变量管理

全局变量是嵌入式工程最容易失控的部分,必须严格管理:

1. **能不用就不用**:优先通过函数参数和返回值传递数据。
2. **能 `static` 就 `static`**:只在单个文件内使用的变量,一律 `static` 限制在本文件,不要 `extern` 出去。
3. **必须全局才全局**:确实需要跨文件共享的,用 `g_模块名_名称` 命名,在对应的 `.c` 中定义,在对应的 `.h` 中用 `extern` 声明。

```c
/* motor_control.c */
uint16_t g_motor_current_speed = 0;   /* 定义 */

/* motor_control.h */
extern uint16_t g_motor_current_speed; /* 声明 */
```

4. **封装访问(推荐)**:对外不直接暴露全局变量,而是提供 getter / setter,既控制写权限,也便于日后重构。

```c
/* motor_control.c */
static uint16_t s_speed = 0;

uint16_t Motor_GetSpeed(void)
{
    return s_speed;
}

void Motor_SetSpeed(uint16_t speed)
{
    s_speed = speed;
}
```

5. **集中初始化**:模块的全局/静态变量在该模块的 `_Init()` 函数中统一初始化,避免"用到哪初始化到哪"。

## 5. 结构体定义风格

- 结构体优先用 `typedef` 定义,类型名以 `_t` 结尾。
- 结构体成员用 snake_case,按"配置/输入/输出/状态"分组,并逐字段加注释:

```c
typedef struct {
    /* 配置 */
    uint16_t max_speed;
    uint8_t  reverse;
    /* 状态 */
    uint16_t current_speed;
    uint8_t  state;
} Motor_t;
```

- 跨模块传递的数据统一用结构体封装,避免一长串散装参数。
- 需要前向引用的结构体,使用带标签的写法:

```c
typedef struct Motor_s {
    struct Motor_s *p_next;
    uint16_t speed;
} Motor_t;
```

## 6. inline 与函数式宏

### 6.1 inline 的使用

- 短小、被频繁调用、且不涉及复杂控制流的工具函数,可用 `static inline` 定义。
- `static inline` 函数通常放在 `.h` 中供多文件复用,或放在单个 `.c` 内作为内部工具:

```c
/* 放在头文件中,供多处复用 */
static inline uint32_t MapValue(uint32_t x, uint32_t in_min, uint32_t in_max,
                                uint32_t out_min, uint32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```

- 不要刻意追求 `inline`;编译器会自动决定是否内联,复杂函数强行 `inline` 无意义。

### 6.2 函数式宏

- **优先用 `static inline` 函数替代函数式宏**,因为函数有类型检查、能避免参数求值两次的副作用问题。
- 仅当必须操作类型无关或编译期常量时才使用函数式宏,且必须给每个参数加括号:

```c
#define CLAMP(x, lo, hi)  (((x) < (lo)) ? (lo) : (((x) > (hi)) ? (hi) : (x)))
#define ARRAY_SIZE(a)     (sizeof(a) / sizeof((a)[0]))
```

- 禁止在函数式宏中做自增、赋值等带副作用的操作。

## 7. 注释规范

### 7.1 文件头注释

每个 `.c` / `.h` 文件开头写文件头注释,说明模块用途:

```c
/**
 * @file    motor_control.c
 * @brief   底盘电机控制模块
 * @author  xxx
 * @date    2025-01-01
 */
```

### 7.2 函数注释

对外接口函数用 Doxygen 风格注释,说明功能、参数、返回值;简单的内部函数可只写一行:

```c
/**
 * @brief   设置电机速度
 * @param   speed  目标速度,单位 rpm
 * @param   dir    方向,取 MOTOR_DIR_FORWARD / MOTOR_DIR_BACKWARD
 * @return  0 成功,-1 失败
 */
int Motor_SetSpeed(uint16_t speed, uint8_t dir);
```

### 7.3 行内注释

- 对非显而易见的逻辑、算法、边界条件、单位、魔法数字必须注释。
- 注释写"为什么",而不是重复"做了什么":

```c
/* 限幅到 PWM 占空比上限,防止烧毁驱动 */
if (duty > MOTOR_MAX_DUTY) {
    duty = MOTOR_MAX_DUTY;
}
```

- 变量声明中的单位、含义可后置注释:

```c
uint16_t motor_speed;   /* rpm */
uint8_t  rx_index;      /* 接收缓冲区写指针 */
```

### 7.4 注释风格约定

- 注释语言统一(建议中文),简洁、准确,与代码同步更新。
- 被注释掉的代码禁止长期保留,提交前应删除或明确标注原因。
- 使用 `/* ... */` 块注释为主;行尾短注释用 `//` 亦可,但同一文件保持一致。

## 8. 其他约定

### 8.1 const 使用

- 不会被修改的形参、局部变量、指针指向的数据,加 `const` 声明意图并让编译器把关:

```c
void Can_SendFrame(const CanFrame_t *p_frame);
static const float g_pid_kp = 1.2f;
```

### 8.2 头文件包含顺序

包含顺序固定,便于排查依赖:

```c
#include "stm32f4xx_hal.h"   /* 芯片/HAL 头文件 */
#include "bsp_uart.h"        /* 本层依赖的模块头 */
#include "motor_control.h"   /* 本文件对应的头文件 */
```

### 8.3 魔法数字

- 直接出现在代码中的数字必须通过 `#define` 或 `enum` 赋予名字,禁止裸数字:

```c
/* 错误 */
if (rx_count > 128) { ... }

/* 正确 */
#define UART_RX_BUF_SIZE 128
if (rx_count > UART_RX_BUF_SIZE) { ... }
```

### 8.4 防御性检查

- 对外接口函数对指针参数、取值范围做必要校验,返回明确错误码,不要默认"调用者一定传对"。

```c
int Motor_SetSpeed(uint16_t speed)
{
    if (speed > MOTOR_MAX_SPEED) {
        return -1;
    }
    s_speed = speed;
    return 0;
}
```

## 9. 反例清单

| 反例 | 问题 | 正例 |
| --- | --- | --- |
| `int a1; int a2;` | 命名无含义 | `int motor_speed; int motor_duty;` |
| 全局变量 `speed` 到处 `extern` | 作用域失控 | `static` 或 `g_motor_speed` + 封装访问 |
| `void f(int x, int y, int z)` | 函数/参数不可读 | `int Motor_SetSpeed(uint16_t speed, uint8_t dir)` |
| `#define MAX 3000` 无前缀 | 易冲突、含义不清 | `#define MOTOR_MAX_SPEED 3000` |
| 裸数字 `if (duty > 3000)` | 魔法数字 | `if (duty > MOTOR_MAX_DUTY)` |
| 一个 `.c` 写满全部逻辑 | 模块不分离 | 按模块拆分为多对 `.c/.h` |
| 头文件无保护宏 | 重复包含报错 | `#ifndef ... #define ... #endif` |
| 函数式宏 `#define SQUARE(x) x*x` | 优先级/副作用问题 | `static inline` 或给参数加括号 |
