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


volatile uint8_t level_flag=1;
volatile uint8_t Is_pick=0;
volatile uint8_t Is_place=0;
volatile uint8_t Is_store=0;
volatile uint8_t Is_ready=0;
volatile uint8_t Is_reset=0;
volatile uint8_t Is_keep=0;
volatile uint8_t Is_on=0;
volatile uint8_t Is_open=0;
volatile uint8_t Is_ok=0;
volatile uint8_t Is_Sys_reset=0;
volatile uint8_t Is_sky_ready=0;

volatile Vec2 Target_Vec;
ArmControl_t ArmControl;
Arm_Interpolation_t Arm_Debug={0,0,0,ARM_Debug_MOVE_TIME};
Arm_Interpolation_Pos_t Arm_Pos_Debug={0,0,0,ARM_Pos_Debug_MOVE_TIME};
Arm_Kinetics_Data_t Arm_Kinetics_Data={0};

static uint8_t Is_enable=0;



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
static float Quintic_Trajectory(float time, float total_time)
{
    float t;
    float t2;
    float t3;
    float t4;
    float t5;
    float s;
    float v0   = ARM_START_VELOCITY;
    float acc0 = ARM_START_ACCEL;
    
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
    return s= v0*t
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

    float s = Quintic_Trajectory(time, total_time);
    return start_angle + (target - start_angle) * s;
}

//开始轨迹计算
static void Arm_Interpolation_Start(float u1_target,float u2_target, float dj_target, float move_time)
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


static void Arm_Interpolation_Pos_Start(float pos_x,float pos_y,float dj_target,float move_time)
{
    Vec2 Pos_target;
    Pos_target.x=pos_x;
    Pos_target.y=pos_y;

    ArmControl.Cart.cartesian = false;
    Unitree_Theta_t angle_target;
    angle_target=Inverse(Pos_target);

    Arm_Interpolation_Start(angle_target.u1_theta,angle_target.u2_theta,dj_target,move_time);
}


static void Arm_Interpolation_Cart_Start(float pos_x, float pos_y,
                                         float dj_target, float move_time)
{
    Unitree_Theta_t now;
    now.u1_theta = Unitree_motors[0].data.position;
    now.u2_theta = Unitree_motors[1].data.position;

    Vec2 now_pos = Forward(now);        // 起点为此刻真实末端位置

    ArmControl.Cart.start_x  = now_pos.x;
    ArmControl.Cart.start_y  = now_pos.y;
    ArmControl.Cart.target_x = pos_x;
    ArmControl.Cart.target_y = pos_y;
    ArmControl.Cart.cartesian = true;

    ArmControl.dj.start_angle = DJmotor[0].valNow.angle_deg;
    ArmControl.dj.target      = dj_target;

    ArmControl.time       = 0.0f;
    ArmControl.total_time = move_time;
    ArmControl.running    = true;
    ArmControl.finish     = false;
}







//轨迹点更新
static void Arm_Interpolation_Update(void)
{
    if (ArmControl.running == false)
    {
        return ;
    }
    
    float s = Quintic_Trajectory(ArmControl.time, ArmControl.total_time);
    if (ArmControl.Cart.cartesian)
    {
       
        Vec2 p;
        p.x = ArmControl.Cart.start_x
            + (ArmControl.Cart.target_x - ArmControl.Cart.start_x) * s;
        p.y = ArmControl.Cart.start_y
            + (ArmControl.Cart.target_y - ArmControl.Cart.start_y) * s;

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
        DJmotor[0].valSet.angle_deg = ArmControl.dj.start_angle+ (ArmControl.dj.target - ArmControl.dj.start_angle) * s;
    }

    else
    {
    Unitree_motors[0].cmd.position =Target_Quintic_Interpolation(ArmControl.u1.start_angle,ArmControl.u1.target,ArmControl.time,ArmControl.total_time);
    Unitree_motors[1].cmd.position = Target_Quintic_Interpolation(ArmControl.u2.start_angle,ArmControl.u2.target,ArmControl.time,ArmControl.total_time);
    DJmotor[0].valSet.angle_deg =  Target_Quintic_Interpolation(ArmControl.dj.start_angle,ArmControl.dj.target,ArmControl.time,ArmControl.total_time);
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
        else{
        ArmControl.time = ArmControl.total_time;
        Unitree_motors[0].cmd.position = ArmControl.u1.target;
        Unitree_motors[1].cmd.position = ArmControl.u2.target;
        }
        DJmotor[0].valSet.angle_deg = ArmControl.dj.target;
        ArmControl.running = false;
        ArmControl.finish = true;
        // Is_pick=0;
        // Is_place=0;
        // Is_store=0;
        // Is_reset=0;
        // Is_ready=0;

    }

}



//机械臂位置、模式初始化
void Arm_Control_Init(void)
{
    ArmControl.state=ARM_STATE_NONE;
    ArmControl.last_state=ARM_STATE_NONE;

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



// void Arm_Control_SetState(ArmState_t state)
// {
//     if (state < ARM_STATE_NONE || state > ARM_STATE_HIGH)
//     {
//         return;
//     }
//      ArmControl.state = state;
// }


// ArmState_t Arm_Control_GetState(void)
// {
//     return ArmControl.state;
// }





//其中一种状态，其他状态最后根据具体情况添加
//准备状态
static void Arm_Ready_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
       Arm_Interpolation_Start(ARM_U1_READY_POS,ARM_U2_READY_POS,ARM_DJ_READY_POS,ARM_MOVE_TIME);
    }
}

//天空块准备阶段
static void Arm_sky_Ready_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
       Arm_Interpolation_Start(ARM_U1_SKY_READY_POS,ARM_U2_SKY_READY_POS,ARM_DJ_SKY_READY_POS,ARM_MOVE_TIME);
    }
}



//调试状态
static void Arm_Debug_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(Arm_Debug.u1_target,Arm_Debug.u2_target,Arm_Debug.dj_target,Arm_Debug.move_time);

    }
}
static void Arm_Pos_Debug_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Pos_Start(Arm_Pos_Debug.x,Arm_Pos_Debug.y,Arm_Pos_Debug.dj_target,Arm_Pos_Debug.move_time);
    
    }
}


//起始状态
static void Arm_NONE_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_START_POS,ARM_U2_START_POS,ARM_DJ_START_POS,ARM_MOVE_TIME);
    }
}

//底层取块
static void Arm_LOW_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_LOW_POS,ARM_U2_LOW_POS,ARM_DJ_LOW_POS,ARM_MOVE_TIME);
    }
}

//二层取块
static void Arm_MID_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_MID_POS,ARM_U2_MID_POS,ARM_DJ_MID_POS,ARM_MOVE_TIME);
    }
}

//取天空块
static void Arm_SKY_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_SKY_POS,ARM_U2_SKY_POS,ARM_DJ_SKY_POS,ARM_MOVE_TIME);
    }
}

// //储存状态
// static void Arm_STORT_Process(void)
// {
//     if (ArmControl.running == false &&
//         ArmControl.finish == false)
//     {
//         Arm_Interpolation_Start(ARM_U1_STORE_POS,ARM_U2_STORE_POS,ARM_DJ_STORE_POS,ARM_MOVE_TIME);
//     }
// }

//底层放块
static void Arm_LOW1_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_LOW1_POS,ARM_U2_LOW1_POS,ARM_DJ_LOW1_POS,ARM_MOVE_TIME);
    }
}

//二层放块
static void Arm_MID1_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_MID1_POS,ARM_U2_MID1_POS,ARM_DJ_MID1_POS,ARM_MOVE_TIME);
    }
}


//三层放块
static void Arm_HIGH_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_HIGH_POS,ARM_U2_HIGH_POS,ARM_DJ_HIGH_POS,ARM_MOVE_SKY_TIME);
    }
}


//持块状态
static void Arm_KEEP_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_KEEP_POS,ARM_U2_KEEP_POS,ARM_DJ_KEEP_POS,ARM_MOVE_TIME);
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

        Arm_Interpolation_Start(ARM_U1_KEEP_POS,ARM_U2_KEEP_POS,ARM_DJ_KEEP_POS,ARM_MOVE_TIME);
    }
}


void Arm_State_Update(void)
{

    if (Is_reset)
    {
        if (ArmControl.state!= ARM_STATE_NONE)
        {
            ArmControl.state = ARM_STATE_NONE;
        }
            Is_pick=0;
            Is_place=0;
            Is_store=0;
            Is_reset=0;
            Is_ready=0;
        return;
    }



    if (Is_ready)
    {
        if(level_flag==0)
        {
            if (ArmControl.state != ARM_STATE_SKY_READY)
            {
            ArmControl.state = ARM_STATE_SKY_READY;
            }
        }
        else if (ArmControl.state != ARM_STATE_READY)
        {
            ArmControl.state = ARM_STATE_READY;
 
        }
            Is_pick=0;
            Is_place=0;
            Is_store=0;
            Is_reset=0;
            Is_ready=0;
        return;
    }

        if (Is_keep)
    {
        if (ArmControl.state != ARM_STATE_KEEP)
        {
            ArmControl.state = ARM_STATE_KEEP;
 
        }
            Is_pick=0;
            Is_place=0;
            Is_store=0;
            Is_reset=0;
            Is_ready=0;
            Is_keep=0;
        return;
    }



    // if (Is_store)
    // {
    //     if (ArmControl.state != ARM_STATE_STORT)
    //     {
    //         ArmControl.state = ARM_STATE_STORT;
    //     }
    //     Is_pick=0;
    //     Is_place=0;
    //     Is_store=0;
    //     Is_ready=0;
    //     Is_reset=0;
    //     return;
    // }


    if (Is_pick==1&&Is_place==0)
    {
        ArmState_t new_state;

        switch (level_flag)
        {
            case 0:
                new_state = ARM_STATE_SKY;
                break;

            case 1:
                new_state = ARM_STATE_LOW;
                break;

            case 2:
                new_state = ARM_STATE_MID;
                break;

            default:
                new_state = ARM_STATE_MID;
                break;
        }


        if (ArmControl.state != new_state)
        {
            ArmControl.state = new_state;

        }
        Is_pick=0;
        Is_place=0;
        Is_store=0;
        Is_ready=0;
        Is_reset=0;

        return;
    }


   
    if (Is_place==1&&Is_pick==0)
    {
        ArmState_t new_state;

        switch (level_flag)
        {
            case 0:
                new_state = ARM_STATE_SKY_READY;
                break;
   

            case 1:
                new_state = ARM_STATE_LOW1;
                break;

            case 2:
                new_state = ARM_STATE_MID1;
                break;

            case 3:
                new_state = ARM_STATE_HIGH;
                break;

            default:
            new_state = ARM_STATE_LOW1;
                break;
        }


        if (ArmControl.state != new_state)
        {
            ArmControl.state = new_state;
           
        }

        Is_pick=0;
        Is_place=0;
        Is_store=0;
        Is_ready=0;
        Is_reset=0;

        return;
    }


    Is_pick=0;
    Is_place=0;
    Is_store=0;
    Is_ready=0;
    Is_reset=0;
    Is_keep=0;
}





//状态更新
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

        Arm_Kinetics_Data.motor_target=Arm_Debug;
        Arm_Kinetics_Data.arm_angle.Angle1=U1_Motor2Geom(Unitree_motors[0].data.position);
        Arm_Kinetics_Data.arm_angle.Angle2=U2_Motor2Geom(Unitree_motors[0].data.position,Unitree_motors[1].data.position);
        Arm_Kinetics_Data.forward_angle.u1_theta=Unitree_motors[0].data.position;
        Arm_Kinetics_Data.forward_angle.u2_theta=Unitree_motors[1].data.position;
        Arm_Kinetics_Data.end_coordinate=Forward(Arm_Kinetics_Data.forward_angle);
        Arm_Kinetics_Data.inverse_angle=Inverse(Arm_Kinetics_Data.end_coordinate);
         if (ArmControl.state != ArmControl.last_state)
        {
             ArmControl.running = false;
            ArmControl.finish = false;
             ArmControl.last_state = ArmControl.state;
        }



        if(Is_Sys_reset==1)
        {
            Sys_reset();
            Is_Sys_reset=0;
        }

        if(Is_open)
        {
            Relay_ON();
        }
        else{
            Relay_OFF();
        }


        if(Is_on==1)
        {
            if(Is_enable==0)
            {
           Arm_Motor_Enable();
           Is_enable=1;
            }
        }

        else{
            if(Is_enable==1)
            {
           Arm_Motor_Disable();
           Is_enable=0;
            }
        }


        switch (ArmControl.state)
        {
            case ARM_STATE_NONE:
                Arm_NONE_Process();
                break;


            case ARM_STATE_READY:

                Arm_Ready_Process();

                break;


            case ARM_STATE_SKY_READY:

                Arm_sky_Ready_Process();
                break;


            case ARM_STATE_KEEP:
                Arm_KEEP_Process();
                break;

            case ARM_STATE_LOW:
                Arm_LOW_Process();
                break;
            

            case ARM_STATE_LOW1:
                Arm_LOW1_Process();
                break;


            case ARM_STATE_MID:
                Arm_MID_Process();
                break;



            case ARM_STATE_MID1:
                Arm_MID1_Process();
                break;


            case ARM_STATE_HIGH:
                Arm_HIGH_Process();
                break;

            case ARM_STATE_SKY:
                Arm_SKY_Process();
                break;

            case ARM_DEBUG:
                Arm_Debug_Process();
                break;

            case ARM_Pos_DEBUG:
                Arm_Pos_Debug_Process();
                break;

            default:
            ArmControl.running = false;
                ArmControl.finish = true;

                break;
        }
        if(Is_on==1)
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

           
                    //使能失能（改）
                   case 0x01020211U:

                       if (Rx_data[0] == 'E')
                       {
                        Is_on=1;
                       }
                       else if (Rx_data[0] == 'D')
                       {
                        Is_on  =0;
                        Is_open=0;
                       }

                       break;

                                           //系统复位
                    case 0x010202F0U:

                       if (Rx_data[0] == 'R')
                       {
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;
                            Is_keep=0U;
                           Is_pick = 0U;
                           Is_Sys_reset=1U;
                       }

                       break; 


                   default:

                       break;
                }



                if(Is_on==1)
                {
               
                 switch (Rxheader.Identifier)
                    {

                    //启动开始，变为持块状态
                    case 0x010202FFU:

                       if (Rx_data[0] == 3)
                       {
                       Is_keep=1;
                       //Is_ready=1;
                       }                     

                       break;





                   //位置调节（改）
                   case 0x01020002U:

                       if (Rx_data[0] <= 3U)
                       {
                           level_flag = Rx_data[0];
                       }

                       break;


                   //返回零位（改）
                   case 0x01020200U:

                       if (Rx_data[0] == 3)
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;

                           Is_reset = 1U;
                           Is_open=0U;
                           
                       }

                       break;



                    //取块准备（改）
                   case 0x01020301U:

                       if (Rx_data[0] == 'P')
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_reset = 0U;

                           Is_ready = 1U;
                       }

                       break;




                   //取块开始(改)
                   case 0x01020307U:

                       if (Rx_data[0] == 'T')
                       {
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_pick = 1U;
                           Is_open=1U;
                       }

                       break;

                    //取块完成（改）
                    case 0x01020309U:

                       if (Rx_data[0] == 'H')
                       {
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;
                           Is_keep =1U;

                           Is_pick = 0U;
                       }

                       break;



                   //放块准备（改）
                   case 0x01020308U:

                       if (Rx_data[0] == 'F')
                       {
                           Is_pick  = 0U;
                           Is_place = 1U;
                           Is_store = 0U;
                           Is_reset = 0U;

                           Is_ready = 0U;
                       }

                       break;


                    //放块开始（改）
                   case 0x01020305U:

                       if (Rx_data[0] == 'R')
                       {
                           Is_pick  = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_place = 0U;
                           Is_open=0U;
                       }

                       break;



                    //放块完成（改）
                    case 0x0102030AU:

                       if (Rx_data[0] == 'O')
                       {
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;
                            Is_keep=1U;
                           Is_pick = 0U;
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