#include "includes.h"
typedef struct { float position, speed, torque; int error; } MotorValues;
typedef struct { MotorValues cmd, data; } TestMotor;
typedef struct { uint32_t last_valid_rx_ms, rx_count, seen; } TestLink;
typedef struct { float angle_deg; } TestDJValue;
typedef struct { TestDJValue valSet, valNow; } TestDJ;
typedef struct { uint32_t state; float total_time; } TestArm;
extern TestMotor Unitree_motors[2];
extern volatile TestLink Unitree_link[2];
extern TestDJ DJmotor[1];
extern TestArm ArmControl;
extern volatile uint8_t Is_on, Is_open;
