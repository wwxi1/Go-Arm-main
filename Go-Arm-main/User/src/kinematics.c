#include "kinematics.h"

/* ============================================================
 * 二自由度平面机械臂运动学
 *
 * 机械结构: 小臂电机与大臂电机同轴安装,小臂经同步带/连杆由小臂
 * 电机驱动(同轴差动)。因此大臂转动时,小臂电机必须同步转相同角度,
 * 才能保持大小臂夹角不变。
 *
 * 角度约定(标定后若符号反了,只需修改下面四个转换函数):
 *   大臂几何角 theta1 = u1 + ARM_U1_ZERO_POS
 *   小臂几何角 theta2 = (u1 - u2) * ARM_U2_RATIO + ARM_U2_ZERO_POS
 *
 *   theta1: 大臂与 +x 轴夹角(方向与 u1 一致)
 *   theta2: 小臂相对大臂的夹角(肘关节角,方向与 u2 相反)
 *
 * 末端坐标(mm):
 *   x = L1*cos(theta1) + L2*cos(theta1+theta2)
 *   y = L1*sin(theta1) + L2*sin(theta1+theta2)
 * ============================================================ */

// 电机角 -> 几何角
float U1_Motor2Geom(float u1)
{
    return u1 + ARM_U1_ZERO_POS;
}

float U2_Motor2Geom(float u1, float u2)
{
    return ARM_U2_ZERO_POS-(u1 + u2) * ARM_U2_RATIO ;
}

// 几何角 -> 电机角
static float U1_Geom2Motor(float theta1)
{
    return theta1 - ARM_U1_ZERO_POS;
}

static float U2_Geom2Motor(float theta1, float theta2)
{
    float u1 = U1_Geom2Motor(theta1);
    return (ARM_U2_ZERO_POS-theta2)/ARM_U2_RATIO-u1;
}

// 正运动学: 关节电机角 -> 末端坐标(mm)
Vec2 Forward(Unitree_Theta_t u_theta)
{
    Vec2 pos;
    float theta1 = U1_Motor2Geom(u_theta.u1_theta);
    float theta2 = U2_Motor2Geom(u_theta.u1_theta, u_theta.u2_theta);

    pos.x = ARM_U1_LENTH * cosf(theta1) + ARM_U2_LENTH * cosf(theta1 + theta2);
    pos.y = ARM_U1_LENTH * sinf(theta1) + ARM_U2_LENTH * sinf(theta1 + theta2);

    return pos;
}

// 逆运动学: 末端坐标(mm) -> 关节电机角(rad),取肘部向上解
Unitree_Theta_t Inverse(Vec2 pos)
{
    Unitree_Theta_t theta;
    const float L1 = ARM_U1_LENTH;
    const float L2 = ARM_U2_LENTH;

    float r2 = pos.x * pos.x + pos.y * pos.y;

    // 余弦定理求肘关节角 theta2
    float cos_theta2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_theta2 > 1.0f)  cos_theta2 = 1.0f;   // 超出可达域时夹紧
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    float theta2 =  acosf(cos_theta2);             // theta2 ∈ [0,pi],肘部向上解

    // 求大臂角 theta1
    float beta  = atan2f(pos.y, pos.x);           // 末端方位角
    float alpha = atan2f(L2 * sinf(theta2), L1 + L2 * cos_theta2);
    float theta1 = beta - alpha;

    theta.u1_theta = U1_Geom2Motor(theta1);
    theta.u2_theta = U2_Geom2Motor(theta1, theta2);

    return theta;
}
