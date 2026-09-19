/* Host test of the actual command receive/mapping functions, with hardware mocks. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "arm_command_under_test.h"

static void reset_test(void)
{
    arm_cmd = ARM_CMD_NONE;
    ArmControl.state = ARM_Pos_DEBUG;
    level_flag = 1;
    Is_on = 1;
    Is_open = Is_Sys_reset = 0;
    irq_mask = 0;
    inject_command = false;
}

static void send(unsigned id, uint8_t value)
{
    FDCAN_RxHeaderTypeDef header = {FDCAN_EXTENDED_ID, id};
    uint8_t data[8] = {value};
    Arm_Receive(header, data);
}

int main(void)
{
    const unsigned ids[] = {0x01020301U, 0x01020307U, 0x01020308U,
                            0x01020309U, 0x01020200U};
    const uint8_t values[] = {'P', 'T', 'F', 'H', 3};
    const ArmState_t expected[4][5] = {
        {ARM_STATE_SKY_READY, ARM_STATE_SKY, ARM_STATE_SKY_READY, ARM_STATE_KEEP, ARM_STATE_NONE},
        {ARM_STATE_READY, ARM_STATE_LOW, ARM_STATE_LOW1, ARM_STATE_KEEP, ARM_STATE_NONE},
        {ARM_STATE_READY, ARM_STATE_MID, ARM_STATE_MID1, ARM_STATE_KEEP, ARM_STATE_NONE},
        {ARM_STATE_READY, ARM_STATE_MID, ARM_STATE_HIGH, ARM_STATE_KEEP, ARM_STATE_NONE}
    };
    for (unsigned level = 0; level < 4; ++level) {
        for (unsigned action = 0; action < 5; ++action) {
            reset_test();
            send(0x01020002U, (uint8_t)level);
            send(ids[action], values[action]);
            Arm_State_Update();
            assert(ArmControl.state == expected[level][action]);
            assert(arm_cmd == ARM_CMD_NONE);
            Arm_State_Update();
            assert(ArmControl.state == expected[level][action]);
        }
    }
    /* Every pair of pending action commands: latest one wins. */
    for (unsigned first = 0; first < 5; ++first) {
        for (unsigned last = 0; last < 5; ++last) {
            reset_test();
            send(ids[first], values[first]);
            send(ids[last], values[last]);
            Arm_State_Update();
            assert(ArmControl.state == expected[1][last]);
        }
    }
    reset_test();
    Arm_State_Update();
    assert(ArmControl.state == ARM_Pos_DEBUG); /* No command preserves debug state. */
    send(0x01020307U, 'X');
    send(0x01020002U, 9);
    assert(arm_cmd == ARM_CMD_NONE && level_flag == 1);

    send(0x01020307U, 'T');
    assert(Is_open == 1);
    send(0x01020211U, 'D');
    assert(!Is_on && !Is_open && arm_cmd == ARM_CMD_NONE);
    send(0x01020307U, 'T');
    assert(arm_cmd == ARM_CMD_NONE && !Is_open);
    send(0x01020211U, 'E');
    assert(Is_on == 1 && arm_cmd == ARM_CMD_NONE);
    send(0x01020305U, 'R');
    assert(!Is_open);
    send(0x0102030AU, 'O');
    assert(arm_cmd == ARM_CMD_KEEP);
    Arm_State_Update();
    assert(ArmControl.state == ARM_STATE_KEEP);
    send(0x010202FFU, 3);
    assert(arm_cmd == ARM_CMD_KEEP);
    send(0x010202F0U, 'R');
    assert(Is_Sys_reset && arm_cmd == ARM_CMD_NONE);

    /* Preserve caller's interrupt mask; a command after the snapshot survives. */
    reset_test();
    irq_mask = 1;
    arm_cmd = ARM_CMD_PICK;
    Arm_State_Update();
    assert(irq_mask == 1 && ArmControl.state == ARM_STATE_LOW);
    irq_mask = 0;
    arm_cmd = ARM_CMD_PLACE;
    inject_command = true;
    Arm_State_Update();
    assert(ArmControl.state == ARM_STATE_LOW1 && arm_cmd == ARM_CMD_KEEP);
    Arm_State_Update();
    assert(ArmControl.state == ARM_STATE_KEEP && arm_cmd == ARM_CMD_NONE);
    puts("PASS: 20 level/action mappings, 25 command pairs, idle/debug, enable/reset/pump and command handoff.");
    return 0;
}
