# 工程 C 语言规范（摘要）

完整版见仓库根目录《武汉大学机器人队工程C语言规范.md》。本工程按该规范执行，核心规则如下。

## 1. 文件

- 文件名全小写 + 下划线：`bsp_led.c`、`can_service.c`。
- 从 R2 底盘移植的成熟驱动保留原文件名（`DJmotor.h`、`VescMotor.h`、`ZDrive.h`、`FD_Canqueue.h`、`mathFunc.h`），以降低历史代码 diff 成本。
- 头文件用 `#ifndef MODULE_H` 保护。
- 一个模块一对 `.c/.h`，一个文件只做一件事。

## 2. 命名

- 变量：`snake_case`。
- 静态文件级变量：`s_` 前缀，如 `s_led_device`。
- 全局变量：`g_` 前缀，如 `g_app_control_task_handle`。
- 函数：`模块名_动作`，首字母大写，如 `Motor_SetRef()`、`BspLed_On()`。
- 类型：`模块名_名字_t`，如 `OsTaskConfig_t`。
- 枚举成员：全大写 + 模块前缀。
- 宏：全大写 + 模块前缀。

## 3. 作用域

- 能 `static` 不全局。
- 跨文件才 `extern`，定义在 `.c`，声明在 `.h`。
- 禁止“一个超大头文件到处 include”，按层提供最小头文件。

## 4. 模块边界

| 文件 | 可调用 |
| --- | --- |
| `Application/*` | BSP/Driver/System/Algorithm/RTOS |
| `BSP/*` | HAL、Config |
| `Driver/*` | HAL、Algorithm、Config |
| `System/*` | HAL、CMSIS-OS2 |
| `Algorithm/*` | 仅 C 标准库 |

## 5. 中断与任务

- ISR 中只做：清标志、取数、少量解析、`OsSem_ReleaseFromIsr()`。
- 长业务逻辑放任务，不在 ISR 中阻塞或打印。
- 周期任务用信号量驱动，不在任务里裸写 `HAL_Delay` 作为主控节奏。

## 6. 模板新增代码的最低要求

1. 头文件保护完整；
2. 公开接口有注释；
3. 魔法数字进 `Config` 或文件顶部宏；
4. 编译无 error，warning 尽量清零；
5. 修改 CubeMX 生成文件时只写在 USER CODE 区域内。
