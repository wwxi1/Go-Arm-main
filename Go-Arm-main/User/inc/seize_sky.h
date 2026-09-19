#ifndef SEIZE_SKY_H
#define SEIZE_SKY_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "UnitreeMotor.h"
#include "DJmotor.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "kinematics.h"
#include "fdcan.h"

//  初始状态
#define ARM_U1_START_POS 0.0f
#define ARM_U2_START_POS 0.0f
#define ARM_DJ_START_POS 0.0f

//  准备状态1
#define ARM_U1_READY_POS 1.6f
#define ARM_U2_READY_POS 0.35f
#define ARM_DJ_READY_POS -233.0f

// 天空块准备阶段
#define ARM_U1_SKY_READY_POS 1.643f
#define ARM_U2_SKY_READY_POS -0.15f
#define ARM_DJ_SKY_READY_POS -222.0f

// //  准备状态
// #define ARM_U1_READY_POS    500.f
// #define ARM_U2_READY_POS    400.f
// #define ARM_DJ_READY_POS    0.0f

//  持块状态1
#define ARM_U1_KEEP_POS 1.16f
#define ARM_U2_KEEP_POS 0.72f
#define ARM_DJ_KEEP_POS -270.0f

// //  存贮状态
// #define ARM_U1_STORE_POS    0.64f
// #define ARM_U2_STORE_POS    0.52f
// #define ARM_DJ_STORE_POS    -240.0f

// 取天空块
#define ARM_U1_SKY_POS 1.71f
#define ARM_U2_SKY_POS -0.5f
#define ARM_DJ_SKY_POS -205.0f

//  底层取块
#define ARM_U1_LOW_POS 1.643f
#define ARM_U2_LOW_POS -0.15f
#define ARM_DJ_LOW_POS -222.0f

// 底层放块
#define ARM_U1_LOW1_POS 1.6f
#define ARM_U2_LOW1_POS 0.55f
#define ARM_DJ_LOW1_POS -241.7f

//  中层取块
#define ARM_U1_MID_POS 1.31f
#define ARM_U2_MID_POS -0.02f
#define ARM_DJ_MID_POS -143.7f

// 中层放块
#define ARM_U1_MID1_POS 1.25f
#define ARM_U2_MID1_POS 0.60f
#define ARM_DJ_MID1_POS -153.7f

//  高层放块
#define ARM_U1_HIGH_POS 1.33f
#define ARM_U2_HIGH_POS 1.18f
#define ARM_DJ_HIGH_POS -161.7f

extern volatile uint8_t level_flag;
extern volatile uint8_t Is_open;
extern volatile uint8_t Is_ok;

typedef enum
{
    ARM_STATE_NONE = 0,
    ARM_STATE_READY,     // 准备状态
    ARM_STATE_LOW,       // 底层取块
    ARM_STATE_MID,       // 中层取块状态
    ARM_STATE_SKY,       // 取天空块
    ARM_STATE_KEEP,      // 持块状态
    ARM_STATE_SKY_READY, // 储存状态
    ARM_STATE_LOW1,      // 放一层
    ARM_STATE_MID1,      // 放二层
    ARM_STATE_HIGH,       // 放三层
    ARM_DEBUG,           // 正运动学调试
    ARM_Pos_DEBUG,       //逆运动学关节坐标系调试
    ARM_CartPos_DEBUG,    //逆运动学笛卡尔坐标
    ARM_STATE_COUNT
} ArmState_t;

typedef enum
{
    ARM_MODE_DEFAULT = 0,   // 默认：关节空间直接给电机转角
    ARM_MODE_CARTESIAN      // 笛卡尔：末端坐标轨迹 + 逐周期逆解
} ArmMode_t;

// 配置表条目（下标 = ArmState_t 枚举值）
typedef struct
{
    float u1;         // 宇树电机1转角 (rad)
    float u2;         // 宇树电机2转角 (rad)
    float dj;         // 大疆电机转角 (deg)
    float move_time;  // 运行时间 (s)
} Arm_Pose_t;

// 轨迹控制结构体
typedef struct
{
    float u1_target, u2_target, dj_target;  // 三个电机目标值
    float u1_start,  u2_start,  dj_start;   // 三个电机起点（切换时读当前实际位置）
    float time;        // 运行时间
    float total_time;  // 总运行时间
    float dt;          // 运动更新周期
    float start_x, start_y, target_x, target_y; // 笛卡尔目标（仅 CARTESIAN）
} ArmTrajCtrl_t;

// 轨迹规划结构体
typedef struct
{
    ArmTrajCtrl_t ctrl;   // 控制结构体
    ArmMode_t      mode;  // 运行模式（笛卡尔 / 默认）
} ArmTraj_t;

//机械臂三个电机的引用(指向真实电机对象,由Arm_Control_Init赋值)
typedef struct
{
    UnitreeMotor *U1;
    UnitreeMotor *U2;
    DJMotor      *DJ;
}Arm_Motor;



// 机械臂运动控制结构体（重构）
typedef struct
{
    volatile bool        enable;     // 使能
    volatile bool        reset;      // 复位（系统复位请求）
    volatile ArmState_t  state;      // 当前状态
    volatile ArmState_t  req_state;  // 请求状态（变化沿检测源）

    Arm_Pose_t pose;   // 电机1/2/3 状态配置（来自配置表）
    ArmTraj_t  traj;   // 轨迹规划结构体
    Arm_Motor arm_motor;

    volatile bool running;   // 插补运行中
    volatile bool finish;    // 插补完成
} ArmControl_t;

typedef struct
{
    float u1_target;
    float u2_target;
    float dj_target;
    float move_time;

} Arm_Interpolation_t;

typedef struct
{
    float x;                 //末端x值
    float y;                 //末端y值
    float dj_target;
    float move_time;

} Arm_Interpolation_Pos_t;

typedef struct 
{
    float Angle1;
    float Angle2;
    float Angle3;
}Arm_Angle;



typedef struct 
{
    Arm_Interpolation_t motor_target;
    Arm_Angle arm_angle;
    Unitree_Theta_t forward_angle;
    Unitree_Theta_t inverse_angle;
    Vec2 end_coordinate;
    /* data */
}Arm_Kinetics_Data_t;

extern Arm_Kinetics_Data_t Arm_Kinetics_Data;
extern Arm_Interpolation_t Arm_Debug;
extern Arm_Interpolation_Pos_t Arm_Pos_Debug;
extern ArmControl_t ArmControl;
extern Arm_Pose_t Arm_Pose_Table[ARM_STATE_COUNT];

void Relay_ON(void);
void Relay_OFF(void);
void Arm_Control_Init(void);
void Arm_Func(void);
void Arm_Motor_Enable(void);
void Arm_Motor_Disable(void);
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);

void Arm_Transmit(void);

#endif
