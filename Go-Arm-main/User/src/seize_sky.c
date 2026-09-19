#include "seize_sky.h"
#include "kinematics.h"
#include "DJmotor.h"



#define ARM_INTERPOLATION_DT    0.001f
#define ARM_EPSILON             0.0001f
#define ARM_MOVE_TIME           1.0f
#define ARM_Debug_MOVE_TIME     6.0f
#define ARM_Pos_Debug_MOVE_TIME 30.0f
#define ARM_MOVE_SKY_TIME       2.0f

//五次多项式轨迹参数(归一化时间域 t∈[0,1],末速度/末加速度固定为0)
//起始速度 s'(0) 与起始加速度 s''(0) 可配置;取0时为平滑起步(等价smoothstep)
#define ARM_START_VELOCITY    0.8f
#define ARM_START_ACCEL       4.6f

Arm_Pose_t Arm_Pose_Table[ARM_STATE_COUNT] =
{
    [ARM_STATE_NONE]      = {ARM_U1_START_POS,    ARM_U2_START_POS,    ARM_DJ_START_POS,    ARM_MOVE_TIME},
    [ARM_STATE_READY]     = {ARM_U1_READY_POS,    ARM_U2_READY_POS,    ARM_DJ_READY_POS,    ARM_MOVE_TIME},
    [ARM_STATE_LOW]       = {ARM_U1_LOW_POS,      ARM_U2_LOW_POS,      ARM_DJ_LOW_POS,      ARM_MOVE_TIME},
    [ARM_STATE_MID]       = {ARM_U1_MID_POS,      ARM_U2_MID_POS,      ARM_DJ_MID_POS,      ARM_MOVE_TIME},
    [ARM_STATE_SKY]       = {ARM_U1_SKY_POS,      ARM_U2_SKY_POS,      ARM_DJ_SKY_POS,      ARM_MOVE_TIME},
    [ARM_STATE_KEEP]      = {ARM_U1_KEEP_POS,     ARM_U2_KEEP_POS,     ARM_DJ_KEEP_POS,     ARM_MOVE_TIME},
    [ARM_STATE_SKY_READY] = {ARM_U1_SKY_READY_POS,ARM_U2_SKY_READY_POS,ARM_DJ_SKY_READY_POS,ARM_MOVE_TIME},
    [ARM_STATE_LOW1]      = {ARM_U1_LOW1_POS,     ARM_U2_LOW1_POS,     ARM_DJ_LOW1_POS,     ARM_MOVE_TIME},
    [ARM_STATE_MID1]      = {ARM_U1_MID1_POS,     ARM_U2_MID1_POS,     ARM_DJ_MID1_POS,     ARM_MOVE_TIME},
    [ARM_STATE_HIGH]      = {ARM_U1_HIGH_POS,     ARM_U2_HIGH_POS,     ARM_DJ_HIGH_POS,     ARM_MOVE_SKY_TIME},
    // 三个调试状态：占位，运行时由 Arm_Refresh_Debug_Pose() 写回
    [ARM_DEBUG]           = {0.0f, 0.0f, 0.0f, ARM_Debug_MOVE_TIME},
    [ARM_Pos_DEBUG]       = {0.0f, 0.0f, 0.0f, ARM_Pos_Debug_MOVE_TIME},
    [ARM_CartPos_DEBUG]   = {0.0f, 0.0f, 0.0f, ARM_Pos_Debug_MOVE_TIME},
};

//把三个调试状态的动态目标值写回配置表(非调试状态直接返回)
static void Arm_Refresh_Debug_Pose(ArmState_t state)
{
    if (state != ARM_DEBUG &&
        state != ARM_Pos_DEBUG &&
        state != ARM_CartPos_DEBUG)
    {
        return;
    }

    Arm_Pose_Table[ARM_DEBUG].u1 = Arm_Debug.u1_target;
    Arm_Pose_Table[ARM_DEBUG].u2 = Arm_Debug.u2_target;
    Arm_Pose_Table[ARM_DEBUG].dj = Arm_Debug.dj_target;
    Arm_Pose_Table[ARM_DEBUG].move_time = Arm_Debug.move_time;

    Vec2 p = {Arm_Pos_Debug.x, Arm_Pos_Debug.y};
    Unitree_Theta_t th = Inverse(p);                 // 关节角（非笛卡尔）
    Arm_Pose_Table[ARM_Pos_DEBUG].u1 = th.u1_theta;
    Arm_Pose_Table[ARM_Pos_DEBUG].u2 = th.u2_theta;
    Arm_Pose_Table[ARM_Pos_DEBUG].dj = Arm_Pos_Debug.dj_target;
    Arm_Pose_Table[ARM_Pos_DEBUG].move_time = Arm_Pos_Debug.move_time;

    // 笛卡尔状态：u1/u2 由 x/y 轨迹覆盖，仅写 dj 与时长
    Arm_Pose_Table[ARM_CartPos_DEBUG].dj = Arm_Pos_Debug.dj_target;
    Arm_Pose_Table[ARM_CartPos_DEBUG].move_time = Arm_Pos_Debug.move_time;
}

volatile uint8_t level_flag=1;
volatile uint8_t Is_open=0;
volatile uint8_t Is_ok=0;

ArmControl_t ArmControl;
Arm_Interpolation_t Arm_Debug={0,0,0,ARM_Debug_MOVE_TIME};
Arm_Interpolation_Pos_t Arm_Pos_Debug={0,0,0,ARM_Pos_Debug_MOVE_TIME};
Arm_Kinetics_Data_t Arm_Kinetics_Data={0};



static void Sys_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void Relay_ON(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
}

void Relay_OFF(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
}



//时间归一化,计算轨迹设定角度(五次多项式)
//s(t)=a1*t+a2*t^2+a3*t^3+a4*t^4+a5*t^5, 边界条件:
//  s(0)=0, s(1)=1
//  s'(0)=v0(可配置), s'(1)=0
//  s''(0)=acc0(可配置), s''(1)=0
static float Quintic_S(float time, float total_time)
{
    float t;
    float t2;
    float t3;
    float t4;
    float t5;
    float v0   = ARM_START_VELOCITY;
    float acc0 = ARM_START_ACCEL;

    if (total_time <= 0.0f)
    {
        return 1.0f;
    }

    t=time/total_time;

    if(t<=0.0f)
    {
        t=0.0f;
    }
    else if(t>=1.0f)
    {
        t=1.0;
    }

    t2=t*t;
    t3=t2*t;
    t4=t3*t;
    t5=t4*t;

    //由边界条件解出的系数:
    //a1=v0, a2=acc0/2, a3=10-6v0-1.5acc0, a4=-15+8v0+1.5acc0, a5=6-3v0-0.5acc0
    return v0*t
      +0.5f*acc0*t2
      +(10.0f-6.0f*v0-1.5f*acc0)*t3
      +(-15.0f+8.0f*v0+1.5f*acc0)*t4
      +(6.0f-3.0f*v0-0.5f*acc0)*t5;
}

static float Target_Quintic_Interpolation(float start_angle, float target,
                                          float time, float total_time)
{
    if (total_time<=0)
    {
        return target;
    }

    float s = Quintic_S(time, total_time);
    return start_angle + (target - start_angle) * s;
}

//开始轨迹计算
static void Arm_Interpolation_Start(Arm_Pose_t pose)
{
    ArmControl.pose = pose;                              // 更新状态配置（电机1/2/3 + 运行时间）

    ArmControl.traj.ctrl.u1_start = Unitree_motors[0].data.position;
    ArmControl.traj.ctrl.u2_start = Unitree_motors[1].data.position;
    ArmControl.traj.ctrl.dj_start = DJmotor[0].valNow.angle_deg;

    ArmControl.traj.ctrl.u1_target = pose.u1;
    ArmControl.traj.ctrl.u2_target = pose.u2;
    ArmControl.traj.ctrl.dj_target = pose.dj;

    ArmControl.traj.ctrl.time       = 0.0f;
    ArmControl.traj.ctrl.total_time = pose.move_time;
    ArmControl.traj.ctrl.dt         = ARM_INTERPOLATION_DT;
    ArmControl.traj.mode            = ARM_MODE_DEFAULT;

    /* 不在此置 running/finish —— 由 Arm_Func 依据目标值变化沿决定 */
}


static void Arm_Interpolation_Cart_Start(Arm_Pose_t pose, float x, float y)
{
    Unitree_Theta_t now = {Unitree_motors[0].data.position,
                           Unitree_motors[1].data.position};
    Vec2 now_pos = Forward(now);                       // 起点 = 当前真实末端

    ArmControl.pose = pose;                            // dj + move_time 来自表
    ArmControl.traj.ctrl.start_x  = now_pos.x;
    ArmControl.traj.ctrl.start_y  = now_pos.y;
    ArmControl.traj.ctrl.target_x = x;
    ArmControl.traj.ctrl.target_y = y;
    ArmControl.traj.ctrl.dj_start = DJmotor[0].valNow.angle_deg;
    ArmControl.traj.ctrl.dj_target = pose.dj;

    ArmControl.traj.ctrl.time = 0.0f;
    ArmControl.traj.ctrl.total_time = pose.move_time;
    ArmControl.traj.ctrl.dt = ARM_INTERPOLATION_DT;
    ArmControl.traj.mode = ARM_MODE_CARTESIAN;

    /* 不在此置 running/finish —— 由 Arm_Func 依据目标坐标变化沿决定 */
}

//轨迹点更新
static void Arm_Interpolation_Update(void)
{
    if (ArmControl.running == false)
    {
        return ;
    }
    
    float s = Quintic_S(ArmControl.traj.ctrl.time, ArmControl.traj.ctrl.total_time);
    if (ArmControl.traj.mode==ARM_MODE_CARTESIAN)
    {

        Vec2 p;
        p.x = ArmControl.traj.ctrl.start_x
            + (ArmControl.traj.ctrl.target_x - ArmControl.traj.ctrl.start_x) * s;
        p.y = ArmControl.traj.ctrl.start_y
            + (ArmControl.traj.ctrl.target_y - ArmControl.traj.ctrl.start_y) * s;

        //可达域检查
        float r = sqrtf(p.x * p.x + p.y * p.y);
        if (r > (ARM_U1_LENTH + ARM_U2_LENTH) ||
            r < fabsf(ARM_U1_LENTH - ARM_U2_LENTH))
        {
            ArmControl.running = false;
            ArmControl.finish  = true;
            return;
        }

        Unitree_Theta_t th = Inverse(p);
        Unitree_motors[0].cmd.position = th.u1_theta;
        Unitree_motors[1].cmd.position = th.u2_theta;
        DJmotor[0].valSet.angle_deg = ArmControl.traj.ctrl.dj_start
            + (ArmControl.traj.ctrl.dj_target - ArmControl.traj.ctrl.dj_start) * s;
    }

    else
    {
    Unitree_motors[0].cmd.position =Target_Quintic_Interpolation(ArmControl.traj.ctrl.u1_start,ArmControl.traj.ctrl.u1_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
    Unitree_motors[1].cmd.position = Target_Quintic_Interpolation(ArmControl.traj.ctrl.u2_start,ArmControl.traj.ctrl.u2_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
    DJmotor[0].valSet.angle_deg =  Target_Quintic_Interpolation(ArmControl.traj.ctrl.dj_start,ArmControl.traj.ctrl.dj_target,ArmControl.traj.ctrl.time,ArmControl.traj.ctrl.total_time);
    }

    ArmControl.traj.ctrl.time += ArmControl.traj.ctrl.dt;

    if (ArmControl.traj.ctrl.time >= ArmControl.traj.ctrl.total_time)
    {
        ArmControl.traj.ctrl.time = ArmControl.traj.ctrl.total_time;
        if (ArmControl.traj.mode==ARM_MODE_CARTESIAN)
        {
            Vec2 p_end;
            p_end.x = ArmControl.traj.ctrl.target_x;
            p_end.y = ArmControl.traj.ctrl.target_y;
            Unitree_Theta_t last = Inverse(p_end);
            Unitree_motors[0].cmd.position = last.u1_theta;
            Unitree_motors[1].cmd.position = last.u2_theta;
        }
        else{
        Unitree_motors[0].cmd.position = ArmControl.traj.ctrl.u1_target;
        Unitree_motors[1].cmd.position = ArmControl.traj.ctrl.u2_target;
        }
        DJmotor[0].valSet.angle_deg = ArmControl.traj.ctrl.dj_target;
        ArmControl.running = false;
        ArmControl.finish = true;
    }

}


//机械臂位置、模式初始化
void Arm_Control_Init(void)
{
    ArmControl.enable    = false;
    ArmControl.reset     = false;
    ArmControl.state     = ARM_STATE_NONE;
    ArmControl.req_state = ARM_STATE_NONE;

    ArmControl.pose = Arm_Pose_Table[ARM_STATE_NONE];

    ArmControl.traj.mode = ARM_MODE_DEFAULT;

    ArmControl.traj.ctrl.u1_start = 0.0f;
    ArmControl.traj.ctrl.u2_start = 0.0f;
    ArmControl.traj.ctrl.dj_start = 0.0f;

    //目标值与初始状态(NONE)配置一致,保证首次非NONE命令能触发目标变化沿
    ArmControl.traj.ctrl.u1_target = Arm_Pose_Table[ARM_STATE_NONE].u1;
    ArmControl.traj.ctrl.u2_target = Arm_Pose_Table[ARM_STATE_NONE].u2;
    ArmControl.traj.ctrl.dj_target = Arm_Pose_Table[ARM_STATE_NONE].dj;

    ArmControl.traj.ctrl.time       = 0.0f;
    ArmControl.traj.ctrl.total_time = ARM_MOVE_TIME;
    ArmControl.traj.ctrl.dt         = ARM_INTERPOLATION_DT;

    ArmControl.traj.ctrl.start_x  = 0.0f;
    ArmControl.traj.ctrl.start_y  = 0.0f;
    ArmControl.traj.ctrl.target_x = 0.0f;
    ArmControl.traj.ctrl.target_y = 0.0f;

    ArmControl.running = false;
    ArmControl.finish  = true;   //防止上电后立刻乱动
}



//机械臂运动更新(1kHz,运行于TIM2中断)
//1.使能沿收敛  2.复位则系统复位  3.夹爪继电器  4.未使能直接return
//5.模式切换变化沿检测(更新状态+配置)  6.运动更新(默认给转角/笛卡尔先规划再逆解)
void Arm_Func(void)
{
    static uint8_t s_motor_enabled = 0;

    /* 1) 使能判断:先收敛电机使能沿 */
    if (ArmControl.enable)
    {
        if (s_motor_enabled == 0)
        {
            Arm_Motor_Enable();
            s_motor_enabled = 1;
        }
    }
    else
    {
        if (s_motor_enabled == 1)
        {
            Arm_Motor_Disable();
            s_motor_enabled = 0;
        }
    }

    /* 2) 复位:直接调用系统复位函数(不返回) */
    if (ArmControl.reset)
    {
        Sys_reset();
        return;
    }

    /* 夹爪继电器(沿用原逻辑:不受使能门控,失能时也能断开) */
    if (Is_open)
    {
        Relay_ON();
    }
    else{
        Relay_OFF();
    }

    /* 未使能:不执行状态切换与运动更新,直接return */
    if (ArmControl.enable == false)
    {
        return;
    }

    /* 3) 模式切换变化沿检测 */
    if (ArmControl.req_state != ArmControl.state)
    {
        Arm_Pose_t pose;
        bool target_changed = false;

        ArmControl.state = ArmControl.req_state;
        Arm_Refresh_Debug_Pose(ArmControl.state);      //调试状态目标值动态写回配置表

        pose = Arm_Pose_Table[ArmControl.state];

        if (ArmControl.state == ARM_CartPos_DEBUG)
        {
            //笛卡尔模式:末端目标坐标发生改变才启动
            target_changed = (ArmControl.traj.ctrl.target_x != Arm_Pos_Debug.x) ||
                             (ArmControl.traj.ctrl.target_y != Arm_Pos_Debug.y);
            Arm_Pos_Debug.x=ArmControl.traj.ctrl.target_x;
            Arm_Pos_Debug.y=ArmControl.traj.ctrl.target_y;
            Arm_Interpolation_Cart_Start(pose, Arm_Pos_Debug.x, Arm_Pos_Debug.y);
        }
        else
        {
            //默认模式:三个电机转角目标值发生改变才启动
            target_changed = (ArmControl.traj.ctrl.u1_target != pose.u1) ||
                             (ArmControl.traj.ctrl.u2_target != pose.u2) ||
                             (ArmControl.traj.ctrl.dj_target != pose.dj);
            pose.u1=ArmControl.traj.ctrl.u1_start;
            pose.u2=ArmControl.traj.ctrl.u2_target;
            pose.dj=ArmControl.traj.ctrl.dj_target;
            Arm_Interpolation_Start(pose);
        }

        if (target_changed)
        {
            ArmControl.traj.ctrl.time = 0.0f;           //重置已运行的时间
            ArmControl.running = true;
            ArmControl.finish  = false;
        }
    }

    /* 4) 运动更新 */
    if (ArmControl.running)
    {
        Arm_Interpolation_Update();
    }

    /* 运动学遥测(1kHz刷新,供VOFA/调试器观察) */
    Arm_Kinetics_Data.motor_target = Arm_Debug;
    Arm_Kinetics_Data.arm_angle.Angle1 = U1_Motor2Geom(Unitree_motors[0].data.position);
    Arm_Kinetics_Data.arm_angle.Angle2 = U2_Motor2Geom(Unitree_motors[0].data.position,Unitree_motors[1].data.position);
    Arm_Kinetics_Data.forward_angle.u1_theta = Unitree_motors[0].data.position;
    Arm_Kinetics_Data.forward_angle.u2_theta = Unitree_motors[1].data.position;
    Arm_Kinetics_Data.end_coordinate = Forward(Arm_Kinetics_Data.forward_angle);
    Arm_Kinetics_Data.inverse_angle = Inverse(Arm_Kinetics_Data.end_coordinate);
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


//取块目标状态映射(按层数)
static ArmState_t Arm_Pick_State(uint8_t level)
{
    switch (level)
    {
        case 0:
            return ARM_STATE_SKY;

        case 1:
            return ARM_STATE_LOW;

        case 2:
            return ARM_STATE_MID;

        default:
            return ARM_STATE_MID;
    }
}

//放块准备目标状态映射(按层数)
static ArmState_t Arm_Place_State(uint8_t level)
{
    switch (level)
    {
        case 0:
            return ARM_STATE_SKY_READY;

        case 1:
            return ARM_STATE_LOW1;

        case 2:
            return ARM_STATE_MID1;

        case 3:
            return ARM_STATE_HIGH;

        default:
            return ARM_STATE_LOW1;
    }
}


//上位机命令接收(FDCAN中断):直接置请求状态/使能/复位,由Arm_Func完成变化沿处理
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if (Rxheader.IdType != FDCAN_EXTENDED_ID)
    {
        return;
    }

    //使能/失能 与 系统复位 不受使能状态限制
    switch (Rxheader.Identifier)
    {
        //使能失能
        case 0x01020211U:

            if (Rx_data[0] == 'E')
            {
                ArmControl.enable = true;
            }
            else if (Rx_data[0] == 'D')
            {
                ArmControl.enable = false;
                Is_open = 0U;
            }

            break;

        //系统复位
        case 0x010202F0U:

            if (Rx_data[0] == 'R')
            {
                ArmControl.reset = true;
            }

            break;

        default:

            break;
    }

    if (ArmControl.enable == false)
    {
        return;
    }

    switch (Rxheader.Identifier)
    {
        //位置调节(层数)
        case 0x01020002U:

            if (Rx_data[0] <= 3U)
            {
                level_flag = Rx_data[0];
            }

            break;

        //启动开始,变为持块状态
        case 0x010202FFU:

            if (Rx_data[0] == 3)
            {
                ArmControl.req_state = ARM_STATE_KEEP;
            }

            break;

        //返回零位
        case 0x01020200U:

            if (Rx_data[0] == 3)
            {
                ArmControl.req_state = ARM_STATE_NONE;
                Is_open = 0U;
            }

            break;

        //取块准备
        case 0x01020301U:

            if (Rx_data[0] == 'P')
            {
                ArmControl.req_state = (level_flag == 0U)
                                     ? ARM_STATE_SKY_READY
                                     : ARM_STATE_READY;
            }

            break;

        //取块开始
        case 0x01020307U:

            if (Rx_data[0] == 'T')
            {
                ArmControl.req_state = Arm_Pick_State(level_flag);
                Is_open = 1U;
            }

            break;

        //取块完成
        case 0x01020309U:

            if (Rx_data[0] == 'H')
            {
                ArmControl.req_state = ARM_STATE_KEEP;
            }

            break;

        //放块准备
        case 0x01020308U:

            if (Rx_data[0] == 'F')
            {
                ArmControl.req_state = Arm_Place_State(level_flag);
            }

            break;

        //放块开始(松爪,不改状态)
        case 0x01020305U:

            if (Rx_data[0] == 'R')
            {
                Is_open = 0U;
            }

            break;

        //放块完成
        case 0x0102030AU:

            if (Rx_data[0] == 'O')
            {
                ArmControl.req_state = ARM_STATE_KEEP;
            }

            break;

        default:

            break;
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

//    if( HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&tx_header,tx_data)!=HAL_OK)
//    {
//         Is_ok=1;
//    }
//    HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
// if (ret != HAL_OK) {
//     // 通过串口或调试器查看 ret 的值
//     Is_ok = ret;  // 把具体错误码存下来
// }

HAL_StatusTypeDef ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, tx_data);
if (ret != HAL_OK) {
    Is_ok = ret;
    HAL_FDCAN_StateTypeDef state = hfdcan3.State;   // 观察值：0=RESET, 1=READY, 2=LISTENING, 4=ERROR
    uint32_t err = HAL_FDCAN_GetError(&hfdcan3); // 读取硬件错误码
    // 可通过串口打印出来，或存储在全局变量中调试
}

}