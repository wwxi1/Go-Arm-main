#pragma once
#include "mathFunc.h"

/* ==================== 二自由度机械臂几何参数 ==================== */

// 大臂 / 小臂初始几何角度(rad),标定前的占位值,标定后修改
#define ARM_U1_ZERO_POS    0.0f
#define ARM_U2_ZERO_POS    2.3562f

// 大臂长 / 小臂长(mm)
#define ARM_U1_LENTH 389.5f
#define ARM_U2_LENTH 423.42f

// 小臂角度减速比: 小臂几何角 = (大臂电机角 - 小臂电机角) * ARM_U2_RATIO
#define ARM_U2_RATIO (2.0f / 3.0f)

// 二维向量(末端坐标,单位 mm)
typedef struct
{
    float x;
    float y;
} Vec2;

// 关节电机角度(单位 rad)
typedef struct
{
    float u1_theta; // 大臂电机角
    float u2_theta; // 小臂电机角
} Unitree_Theta_t;

// 正运动学: 电机角度 -> 末端坐标(mm)
Vec2 Forward(Unitree_Theta_t u_theta);

// 逆运动学: 末端坐标(mm) -> 电机角度(rad,肘部向上解)
Unitree_Theta_t Inverse(Vec2 pos);
float U1_Motor2Geom(float u1);
float U2_Motor2Geom(float u1, float u2);