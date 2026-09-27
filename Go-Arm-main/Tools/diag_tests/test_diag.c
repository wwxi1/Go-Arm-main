#include <assert.h>
#include <stdio.h>
#include "seize_sky.h"
#include "arm_diag.h"
#include "vofa.h"
#include "kinematics.h"
#include <math.h>
#if ARM_DIAG_EVENT_ENABLE
#include "EventRecorder.h"
/* 本地拦截记录API，验证交给Event Recorder的数据；不模拟Keil界面和探针。 */
typedef struct { uint32_t id, values[4]; } CapturedEvent;
static CapturedEvent captured[8192];
static uint32_t captured_count;
static bool fail_event_write, update_feedback_on_event;
uint32_t EventRecorderInitialize(uint32_t levels, uint32_t start)
{ (void)levels; (void)start; captured_count = 0; return 1; }
uint32_t EventRecorderEnable(uint32_t levels, uint32_t first, uint32_t last)
{ (void)levels; (void)first; (void)last; return 1; }
uint32_t EventRecorderDisable(uint32_t levels, uint32_t first, uint32_t last)
{ (void)levels; (void)first; (void)last; return 1; }
static uint32_t capture(uint32_t id, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    assert(captured_count < 8192);
    captured[captured_count++] = (CapturedEvent){id & 0xFFFFU, {a,b,c,d}};
    return fail_event_write ? 0U : 1U;
}
uint32_t EventRecord2(uint32_t id, uint32_t a, uint32_t b)
{
    if (update_feedback_on_event && (id & 0xFFU) == ARM_EVT_RX_TIMEOUT) {
        /* 模拟判断后接收中断到来：导出的仍必须是触发超时的旧快照。 */
        Unitree_link[0].last_valid_rx_ms = HAL_GetTick();
        Unitree_motors[0].data.position = 9.0f;
    }
    return capture(id,a,b,0,0);
}
uint32_t EventRecord4(uint32_t id, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ return capture(id,a,b,c,d); }
static const CapturedEvent *last_detail(uint32_t event)
{
    for (uint32_t i = captured_count; i > 0; --i)
        if (captured[i-1].id == (0x3000U | event)) return &captured[i-1];
    assert(0); return NULL;
}
#endif

TestMotor Unitree_motors[2];
volatile TestLink Unitree_link[2];
TestDJ DJmotor[1];
TestArm ArmControl;
volatile uint8_t Is_on, Is_open;
UART_HandleTypeDef huart9 = {9};
static uint32_t now_ms, mask;
static HAL_StatusTypeDef tx_result = HAL_OK;
static uint8_t *dma_ptr;
static uint32_t tx_calls;
static bool complete_immediately;
static uint32_t unreachable_test_calls;
/* Host substitute for the hardware-dependent trajectory module.
 * Integration build uses the actual guarded Cart_Start entry in seize_sky.c. */
bool Arm_TestUnreachableTarget(void)
{
    assert(!Is_on && !ArmControl.running && !Unitree_motors[0].enable && !Unitree_motors[1].enable);
    ++unreachable_test_calls;
    ArmDiag_Record(ARM_EVT_TEST_UNREACHABLE, 913U);
    ArmDiag_End(ARM_RESULT_INVALID_TARGET);
    return true;
}
extern volatile uint32_t vofa_sent_count, vofa_skipped_count, vofa_start_error_count;
uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1; }
void __set_PRIMASK(uint32_t v) { mask = v; }
void SCB_CleanDCache_by_Addr(uint32_t *p, int32_t n)
{ assert(((uintptr_t)p % 32U) == 0U); assert(n == 32); }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    assert(u == &huart9 && n == 28);
    ++tx_calls;
    if (tx_result == HAL_OK) dma_ptr = p;
    if (complete_immediately && tx_result == HAL_OK) VOFA_DMA_TransmitCpltCallback(u);
    return tx_result;
}

static uint32_t event_count(ArmDiagEvent event)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < g_arm_diag.log_count; ++i)
        if (g_arm_diag.log[i].event == (uint32_t)event) ++count;
    return count;
}

static void arrival_begin(void)
{
    Is_on = 1; g_diag_rx_timeout_ms = 10;
    g_diag_clear_request = g_diag_capture_request = 0;
    memset(Unitree_motors, 0, sizeof(Unitree_motors));
    for (uint32_t i = 0; i < 2; ++i) {
        Unitree_link[i] = (TestLink){.last_valid_rx_ms = now_ms, .seen = 1, .rx_count = 1};
        Unitree_motors[i].cmd.position = Unitree_motors[i].data.position = i ? 0.35f : 1.6f;
    }
    ArmControl.state = 1; ArmControl.total_time = 5;
    ArmControl.running = false;
    ArmDiag_Init(); ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
}

static void arrival_step(uint32_t dt, bool receive)
{
    now_ms += dt;
    if (receive) for (uint32_t i = 0; i < 2; ++i) {
        ++Unitree_link[i].rx_count;
        Unitree_link[i].last_valid_rx_ms = now_ms;
    }
    ArmDiag_Tick();
}

static void test_arrival(void)
{
    arrival_begin();
    for (int i = 0; i < 20; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0); /* first check starts dwell */
    arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1);
    for (int i = 0; i < 30; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1); /* no repeated success spam */

    arrival_begin();
    Vec2 goal = Forward((Unitree_Theta_t){1.6f, 0.35f});
    Unitree_Theta_t outside = Inverse((Vec2){goal.x + 10.5f, goal.y});
    Unitree_motors[0].data.position = outside.u1_theta;
    Unitree_motors[1].data.position = outside.u2_theta;
    for (int i = 0; i < 50; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0);
    Unitree_Theta_t inside = Inverse((Vec2){goal.x + 9.5f, goal.y});
    Unitree_motors[0].data.position = inside.u1_theta;
    Unitree_motors[1].data.position = inside.u2_theta;
    for (int i = 0; i < 21; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1);

    arrival_begin();
    for (int i = 0; i < 200; ++i) arrival_step(10, false);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0 && event_count(ARM_EVT_XY_TIMEOUT) == 1);

    arrival_begin();
    for (int i = 0; i < 11; ++i) arrival_step(10, true);
    arrival_step(20, false); /* stale feedback breaks the continuous dwell */
    for (int i = 0; i < 11; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0);
    for (int i = 0; i < 10; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1);

    arrival_begin(); Unitree_motors[1].data.error = 5;
    for (int i = 0; i < 200; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0 && event_count(ARM_EVT_XY_TIMEOUT) == 1);

    arrival_begin();
    ArmControl.state = 2; ArmDiag_Start();
    assert(event_count(ARM_EVT_XY_ABORTED) == 1);
    uint32_t aborted_index = (g_arm_diag.log_next + ARM_DIAG_LOG_COUNT - 2U) % ARM_DIAG_LOG_COUNT;
    assert(g_arm_diag.log[aborted_index].action_state == 1); /* report OLD action */
    for (int i = 0; i < 30; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0);

    arrival_begin(); Is_on = 0; arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_ABORTED) == 1 && event_count(ARM_EVT_XY_SUCCESS) == 0);

    arrival_begin();
    for (int i = 0; i < 11; ++i) arrival_step(10, true);
    arrival_step(500, true); /* debugger/task pause does not count as stable dwell */
    for (int i = 0; i < 10; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 0);

    now_ms = UINT32_MAX - 100U; arrival_begin();
    for (int i = 0; i < 21; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1); /* unsigned clock wrap */
}

static void test_safety_requests(void)
{
    arrival_begin(); Is_on = 0; ArmDiag_Init();
    float original0 = Unitree_motors[0].cmd.position;
    float original1 = Unitree_motors[1].cmd.position;
    g_diag_test_request = 1; arrival_step(10, true);
    assert(unreachable_test_calls == 1 && g_diag_test_request == 0);
    assert(event_count(ARM_EVT_INVALID_TARGET) == 1);
    assert(!Is_on && Unitree_motors[0].cmd.position == original0 && Unitree_motors[1].cmd.position == original1);

    Is_on = 1; g_diag_test_request = 1; arrival_step(10, true);
    assert(unreachable_test_calls == 1 && g_diag_test_request == 0);
    Is_on = 0; Unitree_motors[0].enable = true;
    g_diag_test_request = 2; arrival_step(10, true);
    Unitree_motors[0].enable = false; ArmControl.running = true;
    g_diag_test_request = 1; arrival_step(10, true);
    ArmControl.running = false;
    assert(unreachable_test_calls == 1 && event_count(ARM_EVT_TEST_REFUSED) == 3);

    ArmDiag_Init(); g_diag_test_request = 2; arrival_step(10, true);
    assert(event_count(ARM_EVT_TEST_OFFSET_ARMED) == 1 && g_diag_test_request == 0);
    Is_on = 1; ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    assert(event_count(ARM_EVT_TEST_OFFSET_APPLIED) == 1);
    assert(Unitree_motors[0].cmd.position == original0 && Unitree_motors[1].cmd.position == original1);
    for (int i = 0; i < 200; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_TIMEOUT) == 1 && event_count(ARM_EVT_XY_SUCCESS) == 0);
    uint32_t last = (g_arm_diag.log_next + ARM_DIAG_LOG_COUNT - 1U) % ARM_DIAG_LOG_COUNT;
    assert(g_arm_diag.log[last].event == ARM_EVT_XY_TIMEOUT && g_arm_diag.log[last].detail == 50U);
    /* The next normal action must not inherit the test offset. */
    ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    for (int i = 0; i < 21; ++i) arrival_step(10, true);
    assert(event_count(ARM_EVT_XY_SUCCESS) == 1 && event_count(ARM_EVT_TEST_OFFSET_APPLIED) == 1);

    Is_on = 0; ArmDiag_Init(); g_diag_test_request = 2; arrival_step(10, true);
    g_diag_test_request = 3; arrival_step(10, true);
    Is_on = 1; ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    assert(event_count(ARM_EVT_TEST_CANCELLED) == 1 && event_count(ARM_EVT_TEST_OFFSET_APPLIED) == 0);

    Is_on = 0; ArmDiag_Init(); g_diag_test_request = 2; arrival_step(10, true);
    Is_on = 1; ArmDiag_Start(); ArmDiag_End(ARM_RESULT_SUPERSEDED);
    ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    assert(event_count(ARM_EVT_TEST_OFFSET_APPLIED) == 0);
}

#if ARM_DIAG_EVENT_ENABLE
static void test_event_details(void)
{
    arrival_begin();
    uint32_t before = captured_count;
    for (int i = 0; i < 21; ++i) arrival_step(10, true);
    /* 正常到位只增加一条主事件（可能另有首次使能事件），没有详细行。 */
    for (uint32_t i = before; i < captured_count; ++i)
        assert((captured[i].id & 0xFFU) < ARM_DETAIL_CONTEXT);

    Unitree_motors[0].data.speed = NAN;
    Unitree_motors[1].cmd.position = -0.1234f;
    Unitree_motors[1].data.position = 0.5678f;
    Unitree_motors[1].data.speed = -2.5f;
    Unitree_motors[1].data.torque = 1.234f;
    uint32_t seq = g_arm_diag.sequence;
    before = captured_count;
    ArmDiag_Record(ARM_EVT_MANUAL_SNAPSHOT, 0);
    assert(captured_count - before == 8U && g_arm_diag.sequence == seq + 1U);
    for (uint32_t i = before + 1U; i < captured_count; ++i)
        assert(captured[i].values[0] == g_arm_diag.sequence);
    const CapturedEvent *angles = last_detail(ARM_DETAIL_ANGLES);
    assert(angles->values[1] == 1U && (int32_t)angles->values[2] == -123 && angles->values[3] == 568U);
    const CapturedEvent *load = last_detail(ARM_DETAIL_LOAD);
    assert((int32_t)load->values[2] == -2500 && load->values[3] == 1234U);
    assert((int32_t)captured[before + 4U].values[2] == INT32_MIN);
    fail_event_write = true;
    uint32_t dropped = g_arm_diag.event_dropped;
    ArmDiag_Record(ARM_EVT_MANUAL_SNAPSHOT, 0);
    assert(g_arm_diag.event_dropped - dropped == 8U);
    fail_event_write = false;

    /* 导出使用判定时的快照，即使主事件输出过程中反馈已恢复。 */
    arrival_begin(); Is_on = 0; ArmDiag_Init();
    now_ms += 20;
    Unitree_link[1].last_valid_rx_ms = now_ms;
    update_feedback_on_event = true;
    ArmDiag_Tick();
    update_feedback_on_event = false;
    const CapturedEvent *link = last_detail(ARM_DETAIL_LINK);
    assert(link->values[1] == 0U && link->values[2] == 20U);
    assert(g_arm_diag.first_fault.sample.age_ms[0] == 20U);
    before = captured_count;
    arrival_step(10, true);
    assert(captured_count - before == 3U); /* 恢复主事件、上下文、该电机链路 */

    /* 故意偏移50 mm：导出偏移后的检查目标，不能误记为原始动作目标。 */
    arrival_begin(); Is_on = 0; ArmDiag_Init();
    g_diag_test_request = 2; arrival_step(10, true);
    Is_on = 1; ArmDiag_Start(); ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    for (int i = 0; i < 200; ++i) arrival_step(10, true);
    const CapturedEvent *goal = last_detail(ARM_DETAIL_XY_GOAL);
    const CapturedEvent *feedback = last_detail(ARM_DETAIL_XY_FEEDBACK);
    assert((int32_t)goal->values[1] - (int32_t)feedback->values[1] == 500);
    assert(goal->values[3] == 100U && feedback->values[3] == 1U);
    assert(goal->values[0] == feedback->values[0]);
    assert(g_arm_diag.first_fault.event == ARM_EVT_XY_TIMEOUT);
    before = captured_count;
    arrival_step(10, true); assert(captured_count == before);

    arrival_begin();
    for (int i = 0; i < 200; ++i) arrival_step(10, false);
    assert(last_detail(ARM_DETAIL_XY_FEEDBACK)->values[3] == 0U);
}
#endif
int main(void)
{
    now_ms = 100;
    ArmDiag_Init();
    assert(!g_arm_comm_ok);
    ArmDiagSnapshot s;
    ArmDiag_Snapshot(&s);
    assert(s.age_ms[0] == UINT32_MAX && mask == 0);
    Unitree_link[0] = (TestLink){.last_valid_rx_ms = UINT32_MAX - 4U, .rx_count = 1, .seen = 1};
    now_ms = 3;
    mask = 1;
    ArmDiag_Snapshot(&s);
    assert(s.age_ms[0] == 8 && mask == 1); /* unsigned tick wrap and mask preservation */
    mask = 0;
    now_ms = 200;
    Unitree_link[0] = (TestLink){.last_valid_rx_ms = 200, .rx_count = 2, .seen = 1};
    Unitree_link[1] = (TestLink){.last_valid_rx_ms = 200, .rx_count = 1, .seen = 1};
    Unitree_motors[0].cmd.position = 1.2f;
    Unitree_motors[0].data.position = 1.0f;
    Unitree_motors[1].cmd.position = 2.2f;
    Unitree_motors[1].data.position = 2.0f;
    float ch[6];
    ArmDiag_FillVofa(ch);
    assert(ch[0] == 1.2f && ch[1] == 1.0f && ch[2] == 2.2f && ch[3] == 2.0f);
    ArmControl.state = 3; ArmControl.total_time = 5;
    ArmDiag_Start(); now_ms = 5200; ArmDiag_End(ARM_RESULT_COMMANDS_SENT);
    assert(g_arm_diag.result == ARM_RESULT_COMMANDS_SENT && g_arm_diag.action_elapsed_ms == 5000);
    ArmDiag_Tick();
    Is_on = 1; g_diag_rx_timeout_ms = 100; now_ms += 10;
    ArmDiag_Tick();
    assert(g_arm_diag.first_fault_valid && g_arm_diag.first_fault.event == ARM_EVT_RX_TIMEOUT);
    uint32_t first = g_arm_diag.first_fault.sequence;
    uint32_t seq = g_arm_diag.sequence;
    now_ms += 10; ArmDiag_Tick();
    assert(g_arm_diag.sequence == seq); /* persistent fault is not spammed */
    Unitree_link[0].last_valid_rx_ms = now_ms;
    Unitree_link[1].last_valid_rx_ms = now_ms;
    now_ms += 10; ArmDiag_Tick();
    assert(g_arm_diag.sequence == seq + 2 && g_arm_diag.first_fault.sequence == first);
    assert(g_arm_comm_ok);
    for (int i = 0; i < 30; ++i) ArmDiag_Record(ARM_EVT_MANUAL_SNAPSHOT, (uint32_t)i);
    assert(g_arm_diag.log_count == ARM_DIAG_LOG_COUNT && g_arm_diag.log_next < ARM_DIAG_LOG_COUNT);
    g_diag_clear_request = 1; now_ms += 10; ArmDiag_Tick();
    assert(g_diag_clear_request == 1 && g_arm_diag.first_fault_valid);
    Is_on = 0; now_ms += 10; ArmDiag_Tick();
    assert(g_diag_clear_request == 0 && !g_arm_diag.first_fault_valid);
    g_diag_vofa_profile = 2; Unitree_motors[1].data.speed = 4.0f;
    ArmDiag_FillVofa(ch); assert(ch[0] == 2.2f && ch[2] == 4.0f);

    /* Disabled arm: fresh replies are OK, either motor stale makes bool false.
     * A disconnect produces one event, not a new event on every check. */
    g_diag_rx_timeout_ms = 10U;
    now_ms += 10;
    Unitree_link[0].last_valid_rx_ms = now_ms - 10U;
    Unitree_link[1].last_valid_rx_ms = now_ms - 10U;
    ArmDiag_Tick(); assert(g_arm_comm_ok && Is_on == 0);
    now_ms += 10;
    Unitree_link[1].last_valid_rx_ms = now_ms;
    ArmDiag_Tick(); assert(!g_arm_comm_ok);
    seq = g_arm_diag.sequence;
    now_ms += 10; Unitree_link[1].last_valid_rx_ms = now_ms;
    ArmDiag_Tick(); assert(g_arm_diag.sequence == seq);
    now_ms += 10;
    Unitree_link[0].last_valid_rx_ms = Unitree_link[1].last_valid_rx_ms = now_ms;
    ArmDiag_Tick(); assert(g_arm_comm_ok && g_arm_diag.sequence == seq + 1U);
    g_diag_rx_timeout_ms = 0;
    now_ms += 10; ArmDiag_Tick(); assert(!g_arm_comm_ok);

    for (uint8_t i = 0; i < 6; ++i) assert(VOFA_Channel_Update(i, VOFA_TYPE_FLOAT, &ch[i]));
    assert(!VOFA_Channel_Update(0, VOFA_TYPE_FLOAT, NULL));
    VOFA_Update(); assert(tx_calls == 1);
    uint8_t first_payload[28]; memcpy(first_payload, dma_ptr, 28);
    uint8_t *first_ptr = dma_ptr;
    float changed = 999; VOFA_Channel_Update(0, VOFA_TYPE_FLOAT, &changed);
    VOFA_Update(); assert(tx_calls == 1 && vofa_skipped_count == 1);
    assert(memcmp(first_payload, first_ptr, 28) == 0); /* in-flight frame not overwritten */
    VOFA_DMA_TransmitCpltCallback(&huart9); VOFA_Update();
    assert(tx_calls == 2 && dma_ptr != first_ptr);
    assert((uintptr_t)dma_ptr >= (uintptr_t)first_ptr + 32 ||
           (uintptr_t)first_ptr >= (uintptr_t)dma_ptr + 32);
    assert(memcmp(first_payload, first_ptr, 28) == 0);
    VOFA_DMA_TransmitCpltCallback(&huart9);
    tx_result = HAL_BUSY; VOFA_Update(); assert(vofa_start_error_count == 1);
    tx_result = HAL_OK; complete_immediately = true;
    VOFA_Update(); VOFA_Update(); /* callback before HAL returns must not leave busy stuck */
    assert(vofa_sent_count == 4 && tx_calls == 5);
    test_arrival();
    test_safety_requests();
#if ARM_DIAG_EVENT_ENABLE
    test_event_details();
#endif
    puts("PASS: one-shot safety requests/refusal/cancel/target preservation; XY arrival; communication, snapshots, event ring, DMA");
    return 0;
}
