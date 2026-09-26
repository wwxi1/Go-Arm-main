#include <assert.h>
#include <stdio.h>
#include "seize_sky.h"
#include "arm_diag.h"
#include "vofa.h"

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
int main(void)
{
    now_ms = 100;
    ArmDiag_Init();
    ArmDiagSnapshot s;
    ArmDiag_Snapshot(&s);
    assert(s.age_ms[0] == UINT32_MAX && mask == 0);
    Unitree_link[0] = (TestLink){UINT32_MAX - 4U, 1, 1};
    now_ms = 3;
    mask = 1;
    ArmDiag_Snapshot(&s);
    assert(s.age_ms[0] == 8 && mask == 1); /* unsigned tick wrap and mask preservation */
    mask = 0;
    now_ms = 200;
    Unitree_link[0] = (TestLink){200, 2, 1};
    Unitree_link[1] = (TestLink){200, 1, 1};
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
    Is_on = 1; g_diag_rx_timeout_ms = 100;
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
    for (int i = 0; i < 30; ++i) ArmDiag_Record(ARM_EVT_MANUAL_SNAPSHOT, (uint32_t)i);
    assert(g_arm_diag.log_count == ARM_DIAG_LOG_COUNT && g_arm_diag.log_next < ARM_DIAG_LOG_COUNT);
    g_diag_clear_request = 1; now_ms += 10; ArmDiag_Tick();
    assert(g_diag_clear_request == 1 && g_arm_diag.first_fault_valid);
    Is_on = 0; now_ms += 10; ArmDiag_Tick();
    assert(g_diag_clear_request == 0 && !g_arm_diag.first_fault_valid);
    g_diag_vofa_profile = 2; Unitree_motors[1].data.speed = 4.0f;
    ArmDiag_FillVofa(ch); assert(ch[0] == 2.2f && ch[2] == 4.0f);

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
    puts("PASS: timestamp wrap, snapshots, event ring, fault latch, profiles, DMA ownership/alignment/failure/completion");
    return 0;
}
