#include "arm_diag.h"
#include "seize_sky.h"
#include "kinematics.h"
#include <math.h>
#include <string.h>
#if ARM_DIAG_EVENT_ENABLE
#include "EventRecorder.h"
#endif
/* Event Recorder 用这三个接口取得时间：复用 HAL 的1 ms时钟，不另开定时器。 */
uint32_t EventRecorderTimerSetup(void) { return 1U; }
uint32_t EventRecorderTimerGetFreq(void) { return 1000U; }
uint32_t EventRecorderTimerGetCount(void) { return HAL_GetTick(); }

volatile ArmDiagnostics g_arm_diag; /* 排障时展开：最新快照、事件环形队列；日常只看comm_ok */
volatile bool g_arm_comm_ok = false;
volatile uint32_t g_diag_rx_timeout_ms = 10U; /* 约5个正常回包周期；候选告警值，不停机 */
volatile uint32_t g_diag_capture_request; /* Watch 写1，请求机械臂任务记录一次快照 */
volatile uint32_t g_diag_clear_request;
volatile uint32_t g_diag_vofa_profile;
volatile uint32_t g_diag_test_request;
static bool test_offset_armed, test_offset_active;
static uint32_t last_task_ms, last_sample_ms;
static uint32_t last_enabled;
static uint32_t timeout_latched[2], previous_error[2];
/* 指令发完后单独等反馈到位，不阻塞轨迹任务，也不改电机目标。 */
static struct {
    bool pending, stable;
    uint32_t state, begin_ms, stable_ms, last_check_ms, rx_count[2];
    Unitree_Theta_t final_angles;
    Vec2 final_xy;
} xy_check;

/* 已有检查快照时直接复用，避免判断后再次采样导致记录与触发原因不同。 */
static void ArmDiag_RecordSample(ArmDiagEvent event, uint32_t detail,
                                 const ArmDiagSnapshot *sample);

static void ArmDiag_XYAbort(uint32_t reason)
{
    if (!xy_check.pending) return;
    xy_check.pending = false;
    /* reason: 1=新动作，2=禁用，3=动作取消/异常，4=目标被另行修改 */
    ArmDiag_Record(ARM_EVT_XY_ABORTED, reason);
}

static void ArmDiag_TestRequest(void)
{
    uint32_t request = g_diag_test_request;
    if (!request) return;
    g_diag_test_request = 0U; /* 拒绝也归0，避免之后突然自动执行旧请求 */
    if (Is_on || ArmControl.running || Unitree_motors[0].enable || Unitree_motors[1].enable) {
        ArmDiag_Record(ARM_EVT_TEST_REFUSED, request);
        return;
    }
    if (request == 1U) {
        test_offset_armed = test_offset_active = false;
        if (!Arm_TestUnreachableTarget()) ArmDiag_Record(ARM_EVT_TEST_REFUSED, request);
    } else if (request == 2U) {
        test_offset_armed = true;
        test_offset_active = false;
        ArmDiag_Record(ARM_EVT_TEST_OFFSET_ARMED, 50U);
    } else if (request == 3U) {
        test_offset_armed = test_offset_active = false;
        ArmDiag_Record(ARM_EVT_TEST_CANCELLED, 0U);
    } else {
        ArmDiag_Record(ARM_EVT_TEST_REFUSED, request);
    }
}

static void ArmDiag_XYTick(const ArmDiagSnapshot *sample)
{
    if (!xy_check.pending) return;
    if (!sample->enabled) { ArmDiag_XYAbort(2U); return; }
    if (sample->target[0] != xy_check.final_angles.u1_theta ||
        sample->target[1] != xy_check.final_angles.u2_theta) {
        ArmDiag_XYAbort(4U); return;
    }

    uint32_t now = sample->time_ms;
    bool continuous = now - xy_check.last_check_ms <= 20U;
    xy_check.last_check_ms = now;
    /* 反馈得持续更新，不能拿同一帧反复“确认”；任务长时间没检查也重新计时。 */
    bool new_feedback = sample->rx_count[0] != xy_check.rx_count[0] &&
                        sample->rx_count[1] != xy_check.rx_count[1];
    xy_check.rx_count[0] = sample->rx_count[0];
    xy_check.rx_count[1] = sample->rx_count[1];
    Unitree_Theta_t angles = {sample->position[0], sample->position[1]};
    Vec2 measured = Forward(angles);
    float dx = measured.x - xy_check.final_xy.x;
    float dy = measured.y - xy_check.final_xy.y;
    float distance2 = dx * dx + dy * dy;
    bool feedback_ok = g_arm_comm_ok && new_feedback &&
                       sample->drive_error[0] == 0U && sample->drive_error[1] == 0U;
    bool inside = feedback_ok && isfinite(distance2) &&
                  distance2 <= ARM_DIAG_XY_TOLERANCE_MM * ARM_DIAG_XY_TOLERANCE_MM;
    if (!inside || !continuous) xy_check.stable = false;
    if (inside && !xy_check.stable) {
        xy_check.stable = true;
        xy_check.stable_ms = now;
    }
    uint32_t elapsed = now - xy_check.begin_ms;
    bool arrived = xy_check.stable && now - xy_check.stable_ms >= ARM_DIAG_XY_DWELL_MS;
    if ((arrived && elapsed <= ARM_DIAG_XY_WAIT_MS) || elapsed >= ARM_DIAG_XY_WAIT_MS) {
        /* 事件显示四舍五入到mm的误差；无效反馈用-1(0xFFFFFFFF)表示。 */
        uint32_t error_mm = UINT32_MAX;
        if (feedback_ok && isfinite(distance2) && distance2 < 1.0e12f)
            error_mm = (uint32_t)(sqrtf(distance2) + 0.5f);
        xy_check.pending = false;
        ArmDiag_RecordSample(arrived && elapsed <= ARM_DIAG_XY_WAIT_MS ?
                             ARM_EVT_XY_SUCCESS : ARM_EVT_XY_TIMEOUT, error_mm, sample);
    }
}

void ArmDiag_Snapshot(ArmDiagSnapshot *out)
{
    /* 把此刻的关键变量复制成一份快照。短暂关中断，避免复制到一半回包更新。
     * 关中断期间不做串口发送、事件记录或等待；结束后恢复原中断状态。 */
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

#if ARM_DIAG_EVENT_ENABLE
/* 用定点整数传数据，避免MCU格式化字符串。INT32_MIN表示非有限值或换算溢出。 */
static uint32_t ArmDiag_Scaled(float value, float scale)
{
    float scaled = value * scale;
    if (!isfinite(scaled) || scaled <= -2147483520.0f || scaled >= 2147483520.0f)
        return (uint32_t)INT32_MIN;
    return (uint32_t)(int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static void ArmDiag_Detail4(ArmDiagDetailEvent event, uint32_t seq,
                            uint32_t a, uint32_t b, uint32_t c)
{
    /* 与主事件同用Op显示层级，现有窗口筛选器不用额外打开Detail。 */
    if (!EventRecord4(EventID(EventLevelOp, 0x30U, event), seq, a, b, c))
        ++g_arm_diag.event_dropped;
}

static void ArmDiag_ExportDetails(const ArmDiagRecord *rec)
{
    ArmDiagEvent event = (ArmDiagEvent)rec->event;
    bool link_only = event == ARM_EVT_RX_TIMEOUT || event == ARM_EVT_RX_RECOVERED;
    bool drive = event == ARM_EVT_DRIVE_ERROR;
    bool xy = event == ARM_EVT_XY_TIMEOUT;
    bool full = xy || event == ARM_EVT_MANUAL_SNAPSHOT || event == ARM_EVT_INVALID_TARGET;
    if (!link_only && !drive && !full) return; /* 正常运行、正常到位不追加详细行 */

    const ArmDiagSnapshot *s = &rec->sample;
    uint32_t seq = rec->sequence;
    ArmDiag_Detail4(ARM_DETAIL_CONTEXT, seq, rec->action_state, s->time_ms, g_diag_rx_timeout_ms);
    uint32_t motor = drive ? (rec->detail >> 16) - 1U : rec->detail;
    for (uint32_t i = 0; i < 2U; ++i) {
        if (!full && i != motor) continue;
        ArmDiag_Detail4(ARM_DETAIL_LINK, seq, i, s->age_ms[i], s->drive_error[i]);
        if (link_only) continue;
        ArmDiag_Detail4(ARM_DETAIL_ANGLES, seq, i,
                       ArmDiag_Scaled(s->target[i], 1000.0f), ArmDiag_Scaled(s->position[i], 1000.0f));
        ArmDiag_Detail4(ARM_DETAIL_LOAD, seq, i,
                       ArmDiag_Scaled(s->speed[i], 1000.0f), ArmDiag_Scaled(s->torque[i], 1000.0f));
    }
    if (xy) {
        /* 记录实际用于检查的终点，因此50 mm测试偏移也能在CSV里看见。
         * 反馈失效时仍显示最后的几何估计，但valid=0，不可当作当前真实位置。 */
        Vec2 measured = Forward((Unitree_Theta_t){s->position[0], s->position[1]});
        ArmDiag_Detail4(ARM_DETAIL_XY_GOAL, seq,
                       ArmDiag_Scaled(xy_check.final_xy.x, 10.0f),
                       ArmDiag_Scaled(xy_check.final_xy.y, 10.0f),
                       ArmDiag_Scaled(ARM_DIAG_XY_TOLERANCE_MM, 10.0f));
        ArmDiag_Detail4(ARM_DETAIL_XY_FEEDBACK, seq,
                       ArmDiag_Scaled(measured.x, 10.0f), ArmDiag_Scaled(measured.y, 10.0f),
                       rec->detail != UINT32_MAX);
    }
}
#endif

static void ArmDiag_RecordSample(ArmDiagEvent event, uint32_t detail,
                                 const ArmDiagSnapshot *sample)
{
    /* 一次调用做两件事：RAM队列保存完整快照，Event Recorder显示主事件及所需详细行。
     * event 是“发生了什么”，detail 是附加整数，例如电机编号或动作耗时。
     * 本函数由机械臂任务调用；其他任务应发请求，不要并发操作这份队列。 */
    ArmDiagRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.sample = *sample;
    rec.sequence = ++g_arm_diag.sequence;
    rec.event = (uint32_t)event;
    rec.detail = detail;
    rec.result = g_arm_diag.result;
    bool xy_event = event == ARM_EVT_XY_SUCCESS || event == ARM_EVT_XY_TIMEOUT ||
                    event == ARM_EVT_XY_ABORTED;
    rec.action_state = xy_event ? xy_check.state : rec.sample.state;
    /* 队列满后覆盖最旧记录；first_fault 另外保留第一次故障快照。 */
    g_arm_diag.log[g_arm_diag.log_next] = rec;
    g_arm_diag.log_next = (g_arm_diag.log_next + 1U) % ARM_DIAG_LOG_COUNT;
    if (g_arm_diag.log_count < ARM_DIAG_LOG_COUNT) ++g_arm_diag.log_count;
    if (!g_arm_diag.first_fault_valid &&
        (event == ARM_EVT_INVALID_TARGET || event == ARM_EVT_RX_TIMEOUT || event == ARM_EVT_XY_TIMEOUT ||
         (event == ARM_EVT_DRIVE_ERROR && detail != 0U))) {
        g_arm_diag.first_fault = rec;
        g_arm_diag.first_fault_valid = 1U;
    }
#if ARM_DIAG_EVENT_ENABLE
    uint32_t level = (event == ARM_EVT_INVALID_TARGET || event == ARM_EVT_RX_TIMEOUT ||
                      event == ARM_EVT_DRIVE_ERROR || event == ARM_EVT_XY_TIMEOUT) ? EventLevelError : EventLevelOp;
    /* 0x30 是本项目自定义组件号；event 是事件号。
     * 后两个参数对应 SCVD 的 val1/val2，分别显示动作状态和附加信息。
     * SCVD 只解释显示名称，不负责检测故障。 */
    if (!EventRecord2(EventID(level, 0x30U, event), rec.action_state, detail))
        ++g_arm_diag.event_dropped;
    ArmDiag_ExportDetails(&rec);
#endif
}

void ArmDiag_Record(ArmDiagEvent event, uint32_t detail)
{
    ArmDiagSnapshot sample;
    ArmDiag_Snapshot(&sample);
    ArmDiag_RecordSample(event, detail, &sample);
}

void ArmDiag_Init(void)
{
    /* seize_sky.c 启动时调用一次：初始化记录器，仅启用本项目0x30组件。 */
    memset((void *)&g_arm_diag, 0, sizeof(g_arm_diag));
    g_arm_comm_ok = false;
    memset(timeout_latched, 0, sizeof(timeout_latched));
    memset(previous_error, 0, sizeof(previous_error));
    last_enabled = 0U;
    memset(&xy_check, 0, sizeof(xy_check));
    test_offset_armed = test_offset_active = false;
    g_diag_test_request = 0U;
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
    ArmDiag_XYAbort(1U); /* 上个动作若尚在等到位，先记录它被打断，再开始新的 */
    /* 一次性测试只绑定下一个动作；被替换/取消后也不顺延到后续动作。 */
    test_offset_active = test_offset_armed;
    test_offset_armed = false;
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
    ArmDiagSnapshot final;
    ArmDiag_Snapshot(&final);
    ArmDiag_RecordSample(event, (uint32_t)result, &final);
    if (result == ARM_RESULT_COMMANDS_SENT) {
        xy_check.pending = true;
        xy_check.stable = false;
        xy_check.state = g_arm_diag.action_state;
        xy_check.begin_ms = xy_check.last_check_ms = final.time_ms;
        xy_check.rx_count[0] = final.rx_count[0];
        xy_check.rx_count[1] = final.rx_count[1];
        /* 固定保存终点，后续不能拿变化中的目标或反馈当新的终点。 */
        xy_check.final_angles = (Unitree_Theta_t){final.target[0], final.target[1]};
        xy_check.final_xy = Forward(xy_check.final_angles);
        if (test_offset_active) {
            /* 只改本地验证参考点！不写cmd、动作表、规划器或反馈数据。 */
            xy_check.final_xy.x += 50.0f;
            ArmDiag_Record(ARM_EVT_TEST_OFFSET_APPLIED, 50U);
        }
    } else {
        ArmDiag_XYAbort(3U);
    }
    test_offset_active = false;
}

void ArmDiag_Tick(void)
{
    /* 机械臂任务周期调用：处理手动请求；每10 ms检查状态。
     * 持续故障只在刚出现时记一次，避免每个循环都挤占事件队列。 */
    uint32_t now = HAL_GetTick();
    uint32_t period = now - last_task_ms;
    last_task_ms = now;
    ArmDiag_TestRequest();
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
    /* 判断“现在是否还收到新反馈”：当前时刻减去最后有效回包时刻，与固定阈值比较。
     * 每10 ms更新一次；超过阈值后，还可能等待一个检查周期才反映到bool。 */
    uint32_t limit_ms = g_diag_rx_timeout_ms;
    g_arm_comm_ok = limit_ms != 0U &&
        sample.age_ms[0] != UINT32_MAX && sample.age_ms[1] != UINT32_MAX &&
        sample.age_ms[0] <= limit_ms && sample.age_ms[1] <= limit_ms;
    ArmDiag_XYTick(&sample);
    if (sample.enabled != last_enabled) {
        last_enabled = sample.enabled;
        ArmDiag_RecordSample(sample.enabled ? ARM_EVT_ENABLE : ARM_EVT_DISABLE, sample.enabled, &sample);
    }
    for (uint32_t i = 0; i < 2U; ++i) {
        /* 首次上线前不刷超时；上线后即使禁用机构也能验证断开/恢复事件。 */
        uint32_t stale = limit_ms && sample.age_ms[i] != UINT32_MAX &&
                         sample.age_ms[i] > limit_ms;
        if (stale && !timeout_latched[i]) {
            timeout_latched[i] = 1;
            ArmDiag_RecordSample(ARM_EVT_RX_TIMEOUT, i, &sample);
        } else if (!stale && timeout_latched[i]) {
            timeout_latched[i] = 0;
            /* 关闭监测不能算恢复；只有重新收到足够新鲜的反馈才记恢复事件。 */
            if (limit_ms && sample.age_ms[i] != UINT32_MAX && sample.age_ms[i] <= limit_ms)
                ArmDiag_RecordSample(ARM_EVT_RX_RECOVERED, i, &sample);
        }
        if (sample.rx_count[i] && sample.drive_error[i] != previous_error[i]) {
            previous_error[i] = sample.drive_error[i];
            /* 高16位：电机编号+1；低16位：驱动错误码 */
            if (sample.drive_error[i])
                ArmDiag_RecordSample(ARM_EVT_DRIVE_ERROR, ((i + 1U) << 16) | sample.drive_error[i], &sample);
        }
    }
}

void ArmDiag_FillVofa(float channels[6])
{
    /* VOFA的通道内容在这里选；这里只填数据，UART9发送由VOFA_SendTask完成。
     * 想换一个波形变量，修改对应 channels[n] 的赋值即可，n范围0~5。 */
    ArmDiagSnapshot sample;
    ArmDiag_Snapshot(&sample);
    uint32_t profile = g_diag_vofa_profile;
    if (profile == 1U || profile == 2U) {
        /* profile=1看电机0，=2看电机1：目标、反馈、速度、力矩、age、动作状态。 */
        uint32_t i = profile - 1U;
        channels[0] = sample.target[i];
        channels[1] = sample.position[i];
        channels[2] = sample.speed[i];
        channels[3] = sample.torque[i];
        channels[4] = sample.age_ms[i] == UINT32_MAX ? -1.0f : (float)sample.age_ms[i];
        channels[5] = (float)sample.state;
    } else {
        /* 默认双电机：0/1=电机0目标/反馈角，2/3=电机1目标/反馈角，4/5=反馈力矩。 */
        channels[0] = sample.target[0]; channels[1] = sample.position[0];
        channels[2] = sample.target[1]; channels[3] = sample.position[1];
        channels[4] = sample.torque[0]; channels[5] = sample.torque[1];
    }
}
