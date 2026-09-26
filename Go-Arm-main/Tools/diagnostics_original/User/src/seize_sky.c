#include "seize_sky.h"
#include "kinematics.h"
#include "DJmotor.h"

#define ARM_INTERPOLATION_DT 0.001f
#define ARM_EPSILON 0.0001f
#define ARM_MOVE_TIME 1.0f
#define ARM_Debug_MOVE_TIME 6.0f
#define ARM_Pos_Debug_MOVE_TIME 30.0f
#define ARM_MOVE_SKY_TIME 2.0f

// 五次多项式轨迹参数(归一化时间域 t∈[0,1],末速度/末加速度固定为0)
// 起始速度 s'(0) 与起始加速度 s''(0) 可配置;取0时为平滑起步(等价smoothstep)
#define ARM_START_VELOCITY 0.8f
#define ARM_START_ACCEL 4.6f

volatile uint8_t level_flag = 1;
volatile ArmCommand_t arm_cmd = ARM_CMD_NONE;
volatile uint8_t Is_on = 0;   // 机构使能
volatile uint8_t Is_open = 0; // 气泵开关
volatile uint8_t Is_Sys_reset = 0;

volatile Vec2 Target_Vec;
ArmControl_t ArmControl;
Arm_Interpolation_t Arm_Debug = {0, 0, 0, ARM_Debug_MOVE_TIME};
Arm_Interpolation_Pos_t Arm_Pos_Debug = {0, 0, 0, ARM_Pos_Debug_MOVE_TIME};
Arm_Kinetics_Data_t Arm_Kinetics_Data = {0};
Arm_Interpolation_Pos_t table[10] = {
    {90.095, 299.402, 0.0, 5},      // 返回初始位
    {-385.885, 586.883, -233.0, 5}, // 大地块准备
    {-447.509, 446.617, -222.0, 5}, // 底层取块
    {-299.395, 515.693, -143.7, 5}, // 中层取块
    {-474.523, 335.910, -205.0, 5}, // 取天空块
    {-114.655, 683.097, -270.0, 5}, // 持块／搬运姿态
    {-447.509, 446.617, -222.0, 5}, // 天空块准备
    {-356.299, 634.917, -241.7, 5}, // 底层放块
    {-181.535, 664.000, -153.7, 5}, // 中层放块
    {-88.258, 760.978, -161.7, 5}   // 高层放块
};

static uint8_t Is_enable = 0;

static void Sys_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void Relay_ON(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

void Relay_OFF(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

// 时间归一化,计算轨迹设定角度(五次多项式)
// s(t)=a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5, 边界条件:
//   s(0)=0, s(1)=1
//   s'(0)=v0(可配置), s'(1)=0
//   s''(0)=acc0(可配置), s''(1)=0
static float Quintic_Trajectory(float time, float total_time)
{
    float t;
    float t2;
    float t3;
    float t4;
    float t5;
    float s;
    float v0 = ARM_START_VELOCITY;
    float acc0 = ARM_START_ACCEL;

    t = time / total_time;

    if (t <= 0.0f)
    {
        t = 0.0f;
    }
    else if (t >= 1.0f)
    {
        t = 1.0;
    }

    t2 = t * t;
    t3 = t2 * t;
    t4 = t3 * t;
    t5 = t4 * t;

    // 由边界条件解出的系数:
    // a1=v0, a2=acc0/2, a3=10-6v0-1.5acc0, a4=-15+8v0+1.5acc0, a5=6-3v0-0.5acc0
    return s = v0 * t + 0.5f * acc0 * t2 + (10.0f - 6.0f * v0 - 1.5f * acc0) * t3 + (-15.0f + 8.0f * v0 + 1.5f * acc0) * t4 + (6.0f - 3.0f * v0 - 0.5f * acc0) * t5;
}

static float Target_Quintic_Interpolation(float start_angle, float target,
                                          float time, float total_time)
{
    if (total_time <= 0)
    {
        return target;
    }

    float s = Quintic_Trajectory(time, total_time);
    return start_angle + (target - start_angle) * s;
}

// 开始轨迹计算
static void Arm_Interpolation_Start(float u1_target, float u2_target, float dj_target, float move_time)
{
    ArmControl.u1.start_angle = Unitree_motors[0].data.position;
    ArmControl.u2.start_angle = Unitree_motors[1].data.position;
    ArmControl.dj.start_angle = DJmotor[0].valNow.angle_deg;

    ArmControl.Cart.cartesian = false;
    ArmControl.u1.target = u1_target;
    ArmControl.u2.target = u2_target;
    ArmControl.dj.target = dj_target;

    ArmControl.time = 0.0f;
    ArmControl.total_time = move_time;

    ArmControl.running = true;
    ArmControl.finish = false;
}

static void Arm_Interpolation_Pos_Start(float pos_x, float pos_y, float dj_target, float move_time)
{
    Vec2 Pos_target;
    Pos_target.x = pos_x;
    Pos_target.y = pos_y;

    ArmControl.Cart.cartesian = false;
    Unitree_Theta_t angle_target;
    angle_target = Inverse(Pos_target);

    Arm_Interpolation_Start(angle_target.u1_theta, angle_target.u2_theta, dj_target, move_time);
}

static void Arm_Interpolation_Cart_Start(float pos_x, float pos_y, float dj_target, float move_time)
{
    Unitree_Theta_t now;
    now.u1_theta = Unitree_motors[0].data.position;
    now.u2_theta = Unitree_motors[1].data.position;

    Vec2 now_pos = Forward(now); // 起点为此刻真实末端位置

    ArmControl.Cart.start_x = now_pos.x;
    ArmControl.Cart.start_y = now_pos.y;
    ArmControl.Cart.target_x = pos_x;
    ArmControl.Cart.target_y = pos_y;
    ArmControl.Cart.cartesian = true;

    ArmControl.dj.start_angle = DJmotor[0].valNow.angle_deg;
    ArmControl.dj.target = dj_target;

    ArmControl.time = 0.0f;
    ArmControl.total_time = move_time;
    ArmControl.running = true;
    ArmControl.finish = false;
}

// 轨迹点更新
static void Arm_Interpolation_Update(void)
{
    if (ArmControl.running == false)
    {
        return;
    }

    float s = Quintic_Trajectory(ArmControl.time, ArmControl.total_time);
    if (ArmControl.Cart.cartesian)
    {

        Vec2 p;
        p.x = ArmControl.Cart.start_x + (ArmControl.Cart.target_x - ArmControl.Cart.start_x) * s;
        p.y = ArmControl.Cart.start_y + (ArmControl.Cart.target_y - ArmControl.Cart.start_y) * s;

        // 可达域检查
        float r = sqrtf(p.x * p.x + p.y * p.y);
        if (r > (ARM_U1_LENTH + ARM_U2_LENTH) ||
            r < fabsf(ARM_U1_LENTH - ARM_U2_LENTH))
        {
            ArmControl.running = false;
            ArmControl.finish = true;
            return;
        }

        Unitree_Theta_t th = Inverse(p);
        Unitree_motors[0].cmd.position = th.u1_theta;
        Unitree_motors[1].cmd.position = th.u2_theta;
        DJmotor[0].valSet.angle_deg = ArmControl.dj.start_angle + (ArmControl.dj.target - ArmControl.dj.start_angle) * s;
    }

    else
    {
        Unitree_motors[0].cmd.position = Target_Quintic_Interpolation(ArmControl.u1.start_angle, ArmControl.u1.target, ArmControl.time, ArmControl.total_time);
        Unitree_motors[1].cmd.position = Target_Quintic_Interpolation(ArmControl.u2.start_angle, ArmControl.u2.target, ArmControl.time, ArmControl.total_time);
        DJmotor[0].valSet.angle_deg = Target_Quintic_Interpolation(ArmControl.dj.start_angle, ArmControl.dj.target, ArmControl.time, ArmControl.total_time);
    }

    ArmControl.time += ARM_INTERPOLATION_DT;

    if (ArmControl.time >= ArmControl.total_time)
    {
        if (ArmControl.Cart.cartesian)
        {
            Vec2 p_end;
            p_end.x = ArmControl.Cart.target_x;
            p_end.y = ArmControl.Cart.target_y;
            Unitree_Theta_t last = Inverse(p_end);
            Unitree_motors[0].cmd.position = last.u1_theta;
            Unitree_motors[1].cmd.position = last.u2_theta;
        }
        else
        {
            ArmControl.time = ArmControl.total_time;
            Unitree_motors[0].cmd.position = ArmControl.u1.target;
            Unitree_motors[1].cmd.position = ArmControl.u2.target;
        }
        DJmotor[0].valSet.angle_deg = ArmControl.dj.target;
        ArmControl.running = false;
        ArmControl.finish = true;
    }
}

// 机械臂位置、模式初始化
void Arm_Control_Init(void)
{
    ArmControl.state = ARM_STATE_NONE;
    ArmControl.last_state = ARM_STATE_NONE;

    ArmControl.time = 0.0f;
    ArmControl.total_time = ARM_MOVE_TIME;

    ArmControl.running = false;
    ArmControl.finish = true;

    ArmControl.u1.start_angle = 0.0f;
    ArmControl.u1.target = ARM_U1_START_POS;

    ArmControl.u2.start_angle = 0.0f;
    ArmControl.u2.target = ARM_U2_START_POS;

    ArmControl.dj.start_angle = 0.0f;
    ArmControl.dj.target = ARM_DJ_START_POS;
}


// 调试状态
static void Arm_Debug_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(Arm_Debug.u1_target, Arm_Debug.u2_target, Arm_Debug.dj_target, Arm_Debug.move_time);
    }
}
static void Arm_Pos_Debug_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Pos_Start(Arm_Pos_Debug.x, Arm_Pos_Debug.y, Arm_Pos_Debug.dj_target, Arm_Pos_Debug.move_time);
    }
}

static void Arm_CartPos_Debug_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Cart_Start(Arm_Pos_Debug.x, Arm_Pos_Debug.y, Arm_Pos_Debug.dj_target, Arm_Pos_Debug.move_time);
    }
}

/*
自由决定末端执行器坐标

坐标-》关节角-》关节角转成可驱动值

*/
static void Arm_Position_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {

        Arm_Interpolation_Start(ARM_U1_KEEP_POS, ARM_U2_KEEP_POS, ARM_DJ_KEEP_POS, ARM_MOVE_TIME);
    }
}

void Arm_State_Update(void)
{
    /* 一次取走命令，避免读出后清零时覆盖刚到达的新命令。 */
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    ArmCommand_t command = arm_cmd;
    uint8_t level = level_flag;
    arm_cmd = ARM_CMD_NONE;
    __set_PRIMASK(irq_mask);

    switch (command)
    {
    case ARM_CMD_READY:
        ArmControl.state = (level == 0) ? ARM_STATE_SKY_READY : ARM_STATE_READY;
        break;
    case ARM_CMD_PICK:
        /* 保留旧规则：0天空块、1底层，其余取中层。 */
        ArmControl.state = (level == 0) ? ARM_STATE_SKY :
                           (level == 1) ? ARM_STATE_LOW : ARM_STATE_MID;
        break;
    case ARM_CMD_PLACE:
        switch (level)
        {
        case 0: ArmControl.state = ARM_STATE_SKY_READY; break;
        case 2: ArmControl.state = ARM_STATE_MID1; break;
        case 3: ArmControl.state = ARM_STATE_HIGH; break;
        default: ArmControl.state = ARM_STATE_LOW1; break;
        }
        break;
    case ARM_CMD_KEEP:
        ArmControl.state = ARM_STATE_KEEP;
        break;
    case ARM_CMD_RESET:
        ArmControl.state = ARM_STATE_NONE;
        break;
    case ARM_CMD_NONE:
    default:
        break;
    }
}

// 状态更新
void Arm_Control_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(1);
        // if (ArmControl.running==1)
        // {
        //     ArmControl.state=ArmControl.last_state;
        // }

        Arm_Kinetics_Data.motor_target = Arm_Debug;
        Arm_Kinetics_Data.arm_angle.Angle1 = U1_Motor2Geom(Unitree_motors[0].data.position);
        Arm_Kinetics_Data.arm_angle.Angle2 = U2_Motor2Geom(Unitree_motors[0].data.position, Unitree_motors[1].data.position);
        Arm_Kinetics_Data.forward_angle.u1_theta = Unitree_motors[0].data.position;
        Arm_Kinetics_Data.forward_angle.u2_theta = Unitree_motors[1].data.position;
        Arm_Kinetics_Data.end_coordinate = Forward(Arm_Kinetics_Data.forward_angle);
        Arm_Kinetics_Data.inverse_angle = Inverse(Arm_Kinetics_Data.end_coordinate);
        if (ArmControl.state != ArmControl.last_state)
        {
            ArmControl.running = false;
            ArmControl.finish = false;
            ArmControl.last_state = ArmControl.state;
        }

        if (Is_Sys_reset == 1)
        {
            Sys_reset();
            Is_Sys_reset = 0;
        }

        if (Is_open)
        {
            Relay_ON();
        }
        else
        {
            Relay_OFF();
        }

        if (Is_on == 1)
        {
            if (Is_enable == 0)
            {
                Arm_Motor_Enable();
                Is_enable = 1;
            }
        }

        else
        {
            if (Is_enable == 1)
            {
                Arm_Motor_Disable();
                Is_enable = 0;
            }
        }
        if (ArmControl.state < 10)
        {
            if (ArmControl.running == false &&
                ArmControl.finish == false)
            {
                Arm_Interpolation_Cart_Start(table[ArmControl.state].x, table[ArmControl.state].y, table[ArmControl.state].dj_target, table[ArmControl.state].move_time);
            }
        }
        else
        {
            switch (ArmControl.state)
            {
            case ARM_DEBUG:
                Arm_Debug_Process();
                break;

            case ARM_Pos_DEBUG:
                Arm_Pos_Debug_Process();
                break;
            case ARM_CartPos_DEBUG:
                Arm_CartPos_Debug_Process();
                break;
            default:
                break;
            }
        }
        if (Is_on == 1)
        {

            if (ArmControl.running == true)
            {
                Arm_Interpolation_Update();
            }
        }
    }
}

void Arm_Motor_Enable(void)
{
    Unitree_motors[0].enable = true;
    Unitree_motors[1].enable = true;

    DJmotor[0].Begin = true;

    DJmotor[0].MODE_Set = DJ_Position;
}

void Arm_Motor_Disable(void)
{
    Unitree_motors[0].enable = false;
    Unitree_motors[1].enable = false;
    DJmotor[0].MODE_Set = DJ_Disable;
    DJmotor[0].Begin = false;
}

void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if (Rxheader.IdType == FDCAN_EXTENDED_ID)
    {
        switch (Rxheader.Identifier)
        {

            // 使能失能（改）
        case 0x01020211U:

            if (Rx_data[0] == 'E')
            {
                Is_on = 1;
            }
            else if (Rx_data[0] == 'D')
            {
                Is_on = 0;
                Is_open = 0;
                arm_cmd = ARM_CMD_NONE;
            }

            break;
            // 系统复位
        case 0x010202F0U:

            if (Rx_data[0] == 'R')
            {
                arm_cmd = ARM_CMD_NONE;
                Is_Sys_reset = 1U;
            }

            break;

        default:

            break;
        }
        if (Is_on == 1)
        {

            switch (Rxheader.Identifier)
            {

            // 启动开始，变为持块状态
            case 0x010202FFU:

                if (Rx_data[0] == 3)
                {
                    arm_cmd = ARM_CMD_KEEP;
                }

                break;

            // 位置调节（改）
            case 0x01020002U:

                if (Rx_data[0] <= 3U)
                {
                    level_flag = Rx_data[0];
                }

                break;

            // 返回零位（改）
            case 0x01020200U:

                if (Rx_data[0] == 3)
                {
                    arm_cmd = ARM_CMD_RESET;
                    Is_open = 0U;
                }

                break;

                // 取块准备（改）
            case 0x01020301U:

                if (Rx_data[0] == 'P')
                {
                    arm_cmd = ARM_CMD_READY;
                }

                break;

            // 取块开始(改)
            case 0x01020307U:

                if (Rx_data[0] == 'T')
                {
                    arm_cmd = ARM_CMD_PICK;
                    Is_open = 1U;
                }

                break;

            // 取块完成（改）
            case 0x01020309U:

                if (Rx_data[0] == 'H')
                {
                    arm_cmd = ARM_CMD_KEEP;
                }

                break;

            // 放块准备（改）
            case 0x01020308U:

                if (Rx_data[0] == 'F')
                {
                    arm_cmd = ARM_CMD_PLACE;
                }

                break;

                // 放块开始（改）
            case 0x01020305U:

                if (Rx_data[0] == 'R')
                {
                    Is_open = 0U; // 气泵关闭
                }

                break;

            // 放块完成（改）
            case 0x0102030AU:

                if (Rx_data[0] == 'O')
                {
                    arm_cmd = ARM_CMD_KEEP;
                }

                break;

            default:

                break;
            }
        }
    }
}

void Arm_Transmit(void)
{
    static uint8_t tx_data[1] = {0};
    FDCAN_TxHeaderTypeDef tx_header = {0};

    tx_header.Identifier = 0x03020101U;

    tx_header.IdType = FDCAN_EXTENDED_ID;

    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    tx_header.DataLength = FDCAN_DLC_BYTES_1;

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    tx_data[0] = level_flag;

    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, tx_data);
    if (ret != HAL_OK)
    {
        HAL_FDCAN_StateTypeDef state = hfdcan3.State; // 观察值：0=RESET, 1=READY, 2=LISTENING, 4=ERROR
        uint32_t err = HAL_FDCAN_GetError(&hfdcan3);  // 读取硬件错误码
        // 可通过串口打印出来，或存储在全局变量中调试
    }
}
