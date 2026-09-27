/*** 
 * @Author: haime7900 haime7900@gmail.com
 * @Date: 2026-09-23 22:21:40
 * @LastEditors: haime7900 haime7900@gmail.com
 * @LastEditTime: 2026-09-26 08:53:06
 * @FilePath: \Go_Arm_Final\Go-Arm-main\Go-Arm-main\User\inc\arm_diag.h
 * @Description: 
 * @
 * @Copyright (c) ${2026} by ${Haime}, All Rights Reserved. 
 */

#ifndef ARM_DIAG_H
#define ARM_DIAG_H

#include <stdint.h>
#include <stdbool.h>

/* 工程已自带Event Recorder，无需通过RTE重复安装。 */
#ifndef ARM_DIAG_EVENT_ENABLE
#define ARM_DIAG_EVENT_ENABLE 1
#endif
#define ARM_DIAG_LOG_COUNT 16U
/* 仅验证两只宇树对应的二维位置，不代表夹爪抓取或DJI姿态轴完成。 */
#define ARM_DIAG_XY_TOLERANCE_MM 10.0f
#define ARM_DIAG_XY_DWELL_MS 200U
#define ARM_DIAG_XY_WAIT_MS 2000U

/* 以下是轨迹生成器的结果，不表示机构已安全停止。 */
typedef enum {
    ARM_RESULT_IDLE = 0,
    ARM_RESULT_RUNNING = 1,
    ARM_RESULT_COMMANDS_SENT = 2, /* 指令发完，尚未确认实际到位 */
    ARM_RESULT_INVALID_TARGET = 3,
    ARM_RESULT_CANCELLED = 4,
    ARM_RESULT_SUPERSEDED = 5,
    ARM_RESULT_RESET_REQUESTED = 6
} ArmResult;

typedef enum {
    ARM_EVT_BOOT = 0,
    ARM_EVT_START = 1,
    ARM_EVT_END = 2,
    ARM_EVT_INVALID_TARGET = 3,
    ARM_EVT_CANCEL = 4,
    ARM_EVT_SUPERSEDE = 5,
    ARM_EVT_RESET = 6,
    ARM_EVT_RX_TIMEOUT = 7,
    ARM_EVT_RX_RECOVERED = 8,
    ARM_EVT_DRIVE_ERROR = 9,
    ARM_EVT_ENABLE = 10,
    ARM_EVT_DISABLE = 11,
    ARM_EVT_MANUAL_SNAPSHOT = 12,
    ARM_EVT_XY_SUCCESS = 13,
    ARM_EVT_XY_TIMEOUT = 14,
    ARM_EVT_XY_ABORTED = 15,
    ARM_EVT_TEST_UNREACHABLE = 16,
    ARM_EVT_TEST_OFFSET_ARMED = 17,
    ARM_EVT_TEST_OFFSET_APPLIED = 18,
    ARM_EVT_TEST_REFUSED = 19,
    ARM_EVT_TEST_CANCELLED = 20
} ArmDiagEvent;

/* 只用于Event Recorder的附加行，不再写入RAM事件队列，避免占用16条主事件。
 * 每行的seq关联同一份快照；定点整数的单位由SCVD列名注明。 */
typedef enum {
    ARM_DETAIL_CONTEXT = 21,
    ARM_DETAIL_LINK = 22,
    ARM_DETAIL_ANGLES = 23,
    ARM_DETAIL_LOAD = 24,
    ARM_DETAIL_XY_GOAL = 25,
    ARM_DETAIL_XY_FEEDBACK = 26
} ArmDiagDetailEvent;

typedef struct {
    uint32_t time_ms;         /* 取得这份快照的HAL毫秒时间 */
    uint32_t state;           /* 当前动作模式 */
    uint32_t enabled;         /* 机构使能请求 */
    uint32_t pump_requested;  /* 气泵开关请求 */
    float target[2];          /* 本周期的电机指令角，单位rad */
    float position[2];        /* 扣除软件零位后的反馈角，单位rad */
    float speed[2];           /* 反馈速度，单位rad/s */
    float torque[2];          /* 驱动换算后的反馈力矩，单位N·m */
    uint32_t age_ms[2];       /* 反馈年龄；UINT32_MAX表示从未收到 */
    uint32_t rx_count[2];
    uint32_t drive_error[2];
    float dj_target_deg;
    float dj_position_deg;
} ArmDiagSnapshot;

typedef struct {
    uint32_t sequence;       /* 本轮启动内的主事件序号，对应详细行的seq */
    uint32_t event;          /* 事件编号 */
    uint32_t detail;         /* 电机编号、误差等附加信息，含义取决于事件 */
    uint32_t result;         /* 轨迹生成结果；到位结果另看event */
    ArmDiagSnapshot sample; /* 事件发生时的完整现场，保存在RAM中 */
    uint32_t action_state; /* 事件所属动作；可与新动作开始时的sample.state不同 */
} ArmDiagRecord;

typedef struct {
    uint32_t result;
    uint32_t action_state;
    uint32_t action_start_ms;
    uint32_t action_elapsed_ms;
    uint32_t max_task_period_ms;
    uint32_t event_init_ok;
    uint32_t event_dropped; /* 主事件/详细行写入失败次数，不包括被旧记录覆盖的数量 */
    uint32_t log_next;      /* 环形队列下一次写入下标 */
    uint32_t log_count;     /* 已保存主事件数量，最多16条 */
    uint32_t sequence;
    uint32_t first_fault_valid;
    ArmDiagSnapshot latest;
    ArmDiagRecord first_fault;
    ArmDiagRecord log[ARM_DIAG_LOG_COUNT];
} ArmDiagnostics;

extern volatile ArmDiagnostics g_arm_diag;
/* 日常只看这一项：两台宇树都有有效反馈，且当前反馈年龄均不超过阈值。
 * 不表示机械、电机驱动器或末端都正常；监测关闭/从未收到时也是false。 */
extern volatile bool g_arm_comm_ok;
/* 默认10 ms为实测约2 ms基础上的调试告警候选值，不是已验证的安全期限。
 * 0关闭监测，comm_ok=false；非0仅告警，禁用机械臂时也监测已上线电机。 */
extern volatile uint32_t g_diag_rx_timeout_ms;
extern volatile uint32_t g_diag_capture_request;
extern volatile uint32_t g_diag_clear_request; /* 仅在Is_on=0时接受 */
/* 0：双电机；1：电机0；2：电机1。开始采集波形前选择。 */
extern volatile uint32_t g_diag_vofa_profile;
/* Watch一次性请求，处理后自动归0，只在机构禁用且无运行轨迹时接受：
 * 1=检查固定不可达点；2=下一动作仅验证坐标偏移50 mm；3=取消待执行测试。
 * 请求2不启动动作，仍由操作者按原流程发出已验证的正常动作。 */
extern volatile uint32_t g_diag_test_request;

void ArmDiag_Init(void);
void ArmDiag_Tick(void); /* 仅机械臂任务调用；检查间隔至少10 ms */
void ArmDiag_Snapshot(ArmDiagSnapshot *out);
void ArmDiag_Start(void);
void ArmDiag_End(ArmResult result);
void ArmDiag_Record(ArmDiagEvent event, uint32_t detail);
void ArmDiag_FillVofa(float channels[6]);

#endif
