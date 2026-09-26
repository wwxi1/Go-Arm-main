#include "arm_diag.h"
#include "seize_sky.h"
#include <string.h>
#if ARM_DIAG_EVENT_ENABLE
#include "EventRecorder.h"
#endif
/* Millisecond event time uses the existing HAL timebase, not another timer. */
uint32_t EventRecorderTimerSetup(void) { return 1U; }
uint32_t EventRecorderTimerGetFreq(void) { return 1000U; }
uint32_t EventRecorderTimerGetCount(void) { return HAL_GetTick(); }

volatile ArmDiagnostics g_arm_diag;
volatile uint32_t g_diag_rx_timeout_ms = 0U;
volatile uint32_t g_diag_capture_request;
volatile uint32_t g_diag_clear_request;
volatile uint32_t g_diag_vofa_profile;
static uint32_t last_task_ms, last_sample_ms;
static uint32_t last_enabled;
static uint32_t timeout_latched[2], previous_error[2];

void ArmDiag_Snapshot(ArmDiagSnapshot *out)
{
    /* Short fixed-size copy only: no math, I/O, EventRecorder or waits masked. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    out->time_ms = HAL_GetTick();
    out->state = (uint32_t)ArmControl.state;
    out->enabled = Is_on;
    out->pump_requested = Is_open;
    for (uint32_t i = 0; i < 2U; ++i) {
        out->target[i] = Unitree_motors[i].cmd.position;
        out->position[i] = Unitree_motors[i].data.position;
        out->speed[i] = Unitree_motors[i].data.speed;
        out->torque[i] = Unitree_motors[i].data.torque;
        out->rx_count[i] = Unitree_link[i].rx_count;
        out->age_ms[i] = Unitree_link[i].seen ?
            out->time_ms - Unitree_link[i].last_valid_rx_ms : UINT32_MAX;
        out->drive_error[i] = (uint32_t)Unitree_motors[i].data.error;
    }
    out->dj_target_deg = DJmotor[0].valSet.angle_deg;
    out->dj_position_deg = DJmotor[0].valNow.angle_deg;
    __set_PRIMASK(mask);
}

void ArmDiag_Record(ArmDiagEvent event, uint32_t detail)
{
    ArmDiagRecord rec;
    memset(&rec, 0, sizeof(rec));
    ArmDiag_Snapshot(&rec.sample);
    rec.sequence = ++g_arm_diag.sequence;
    rec.event = (uint32_t)event;
    rec.detail = detail;
    rec.result = g_arm_diag.result;
    /* Only called from init (before task start) or the arm task. */
    g_arm_diag.log[g_arm_diag.log_next] = rec;
    g_arm_diag.log_next = (g_arm_diag.log_next + 1U) % ARM_DIAG_LOG_COUNT;
    if (g_arm_diag.log_count < ARM_DIAG_LOG_COUNT) ++g_arm_diag.log_count;
    if (!g_arm_diag.first_fault_valid &&
        (event == ARM_EVT_INVALID_TARGET || event == ARM_EVT_RX_TIMEOUT ||
         (event == ARM_EVT_DRIVE_ERROR && detail != 0U))) {
        g_arm_diag.first_fault = rec;
        g_arm_diag.first_fault_valid = 1U;
    }
#if ARM_DIAG_EVENT_ENABLE
    uint32_t level = (event == ARM_EVT_INVALID_TARGET || event == ARM_EVT_RX_TIMEOUT ||
                      event == ARM_EVT_DRIVE_ERROR) ? EventLevelError : EventLevelOp;
    if (!EventRecord2(EventID(level, 0x30U, event), rec.sample.state, detail))
        ++g_arm_diag.event_dropped;
#endif
}

void ArmDiag_Init(void)
{
    memset((void *)&g_arm_diag, 0, sizeof(g_arm_diag));
#if ARM_DIAG_EVENT_ENABLE
    g_arm_diag.event_init_ok = EventRecorderInitialize(EventRecordAll, 1U);
    EventRecorderDisable(EventRecordAll, 0x00U, 0xFEU);
    EventRecorderEnable(EventRecordAll, 0x30U, 0x30U);
#endif
    last_task_ms = last_sample_ms = HAL_GetTick();
    ArmDiag_Record(ARM_EVT_BOOT, 0U);
}

void ArmDiag_Start(void)
{
    g_arm_diag.result = ARM_RESULT_RUNNING;
    g_arm_diag.action_state = (uint32_t)ArmControl.state;
    g_arm_diag.action_start_ms = HAL_GetTick();
    g_arm_diag.action_elapsed_ms = 0;
    float duration_ms = ArmControl.total_time * 1000.0f;
    ArmDiag_Record(ARM_EVT_START, duration_ms >= 4294967295.0f ? UINT32_MAX : (uint32_t)duration_ms);
}

void ArmDiag_End(ArmResult result)
{
    ArmDiagEvent event = ARM_EVT_END;
    g_arm_diag.result = result;
    g_arm_diag.action_elapsed_ms = HAL_GetTick() - g_arm_diag.action_start_ms;
    switch (result) {
    case ARM_RESULT_INVALID_TARGET: event = ARM_EVT_INVALID_TARGET; break;
    case ARM_RESULT_CANCELLED: event = ARM_EVT_CANCEL; break;
    case ARM_RESULT_SUPERSEDED: event = ARM_EVT_SUPERSEDE; break;
    case ARM_RESULT_RESET_REQUESTED: event = ARM_EVT_RESET; break;
    default: break;
    }
    ArmDiag_Record(event, (uint32_t)result);
}

void ArmDiag_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t period = now - last_task_ms;
    last_task_ms = now;
    if (period > g_arm_diag.max_task_period_ms) g_arm_diag.max_task_period_ms = period;
    if (g_diag_clear_request && !Is_on) {
        g_diag_clear_request = 0;
        g_arm_diag.first_fault_valid = 0;
        memset((void *)&g_arm_diag.first_fault, 0, sizeof(g_arm_diag.first_fault));
        g_arm_diag.log_count = g_arm_diag.log_next = 0;
        g_arm_diag.max_task_period_ms = 0;
        memset(timeout_latched, 0, sizeof(timeout_latched));
    }
    if (g_diag_capture_request) {
        g_diag_capture_request = 0;
        ArmDiag_Record(ARM_EVT_MANUAL_SNAPSHOT, 0);
    }
    if (now - last_sample_ms < 10U) return;
    last_sample_ms = now;
    ArmDiagSnapshot sample;
    ArmDiag_Snapshot(&sample);
    g_arm_diag.latest = sample;
    if (sample.enabled != last_enabled) {
        last_enabled = sample.enabled;
        ArmDiag_Record(sample.enabled ? ARM_EVT_ENABLE : ARM_EVT_DISABLE, sample.enabled);
    }
    for (uint32_t i = 0; i < 2U; ++i) {
        uint32_t stale = g_diag_rx_timeout_ms && sample.enabled &&
                         sample.age_ms[i] > g_diag_rx_timeout_ms;
        if (stale && !timeout_latched[i]) {
            timeout_latched[i] = 1;
            ArmDiag_Record(ARM_EVT_RX_TIMEOUT, i);
        } else if (!stale && timeout_latched[i]) {
            timeout_latched[i] = 0;
            /* Disable/threshold=0 is not communication recovery. */
            if (g_diag_rx_timeout_ms && sample.age_ms[i] <= g_diag_rx_timeout_ms)
                ArmDiag_Record(ARM_EVT_RX_RECOVERED, i);
        }
        if (sample.rx_count[i] && sample.drive_error[i] != previous_error[i]) {
            previous_error[i] = sample.drive_error[i];
            /* high 16 bits: motor index+1; low 16 bits: error code */
            if (sample.drive_error[i])
                ArmDiag_Record(ARM_EVT_DRIVE_ERROR, ((i + 1U) << 16) | sample.drive_error[i]);
        }
    }
}

void ArmDiag_FillVofa(float channels[6])
{
    ArmDiagSnapshot sample;
    ArmDiag_Snapshot(&sample);
    uint32_t profile = g_diag_vofa_profile;
    if (profile == 1U || profile == 2U) {
        uint32_t i = profile - 1U;
        channels[0] = sample.target[i];
        channels[1] = sample.position[i];
        channels[2] = sample.speed[i];
        channels[3] = sample.torque[i];
        channels[4] = sample.age_ms[i] == UINT32_MAX ? -1.0f : (float)sample.age_ms[i];
        channels[5] = (float)sample.state;
    } else {
        channels[0] = sample.target[0]; channels[1] = sample.position[0];
        channels[2] = sample.target[1]; channels[3] = sample.position[1];
        channels[4] = sample.torque[0]; channels[5] = sample.torque[1];
    }
}
