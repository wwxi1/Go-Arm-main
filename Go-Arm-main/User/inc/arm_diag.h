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

/* Event Recorder is bundled locally; no RTE installation is needed. */
#ifndef ARM_DIAG_EVENT_ENABLE
#define ARM_DIAG_EVENT_ENABLE 1
#endif
#define ARM_DIAG_LOG_COUNT 16U

/* These describe the trajectory generator, NOT physical safe-stop states. */
typedef enum {
    ARM_RESULT_IDLE = 0,
    ARM_RESULT_RUNNING = 1,
    ARM_RESULT_COMMANDS_SENT = 2, /* NOT verified arrival */
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
    ARM_EVT_MANUAL_SNAPSHOT = 12
} ArmDiagEvent;

typedef struct {
    uint32_t time_ms;
    uint32_t state;
    uint32_t enabled;
    uint32_t pump_requested;
    float target[2];          /* rad: this cycle's motor command */
    float position[2];        /* rad: offset-corrected feedback */
    float speed[2];           /* rad/s */
    float torque[2];          /* driver-converted feedback, Nm */
    uint32_t age_ms[2];       /* UINT32_MAX means never received */
    uint32_t rx_count[2];
    uint32_t drive_error[2];
    float dj_target_deg;
    float dj_position_deg;
} ArmDiagSnapshot;

typedef struct {
    uint32_t sequence;
    uint32_t event;
    uint32_t detail;
    uint32_t result;
    ArmDiagSnapshot sample;
} ArmDiagRecord;

typedef struct {
    uint32_t result;
    uint32_t action_state;
    uint32_t action_start_ms;
    uint32_t action_elapsed_ms;
    uint32_t max_task_period_ms;
    uint32_t event_init_ok;
    uint32_t event_dropped;
    uint32_t log_next;
    uint32_t log_count;
    uint32_t sequence;
    uint32_t first_fault_valid;
    ArmDiagSnapshot latest;
    ArmDiagRecord first_fault;
    ArmDiagRecord log[ARM_DIAG_LOG_COUNT];
} ArmDiagnostics;

extern volatile ArmDiagnostics g_arm_diag;
/* Watch controls. Timeout 0 = alarm disabled; NONZERO = observe-only alarm.
 * No automatic motor disable/hold is inferred from this threshold. */
extern volatile uint32_t g_diag_rx_timeout_ms;
extern volatile uint32_t g_diag_capture_request;
extern volatile uint32_t g_diag_clear_request; /* accepted only while Is_on=0 */
/* 0: both joints; 1: motor0; 2: motor1. Select before capturing a plot. */
extern volatile uint32_t g_diag_vofa_profile;

void ArmDiag_Init(void);
void ArmDiag_Tick(void); /* ONLY arm control task; samples at >=10ms */
void ArmDiag_Snapshot(ArmDiagSnapshot *out);
void ArmDiag_Start(void);
void ArmDiag_End(ArmResult result);
void ArmDiag_Record(ArmDiagEvent event, uint32_t detail);
void ArmDiag_FillVofa(float channels[6]);

#endif
