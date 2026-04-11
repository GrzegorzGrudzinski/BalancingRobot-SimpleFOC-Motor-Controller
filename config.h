/*
    config.h

*/

#pragma once

typedef enum {
    ERR_OK,
    ERR_INIT,
    ERR_COMMUNICATION,
    ERR_INV_COMMAND,
    ERR_IS_NAN,
    ERR_COM_TIMEOUT,
    ERR_MOTOR_STALL,   
    ERR_DESYNC,        
    ERR_OPPOSITE_SPIN    
} error_states_t;

extern error_states_t error_state;
extern bool sys_error;

// =====================================
// COMMUNICATION
// =====================================
#define RX_PIN 16  // ESP32 RX Pin 
#define TX_PIN 17  // ESP32 TX Pin
#define UART_NR 2

#define COMM_TIMEOUT_MS 500 // 0.5 sec

// =====================================
// ENCODERS
// =====================================
#define ENC1_A  18
#define ENC1_B  19
#define ENC1_PPR    1000

#define ENC2_A  5
#define ENC2_B  23
#define ENC2_PPR    1000

// =====================================
// MOTORS & DRIVERS
// =====================================
#define M1_PP   7
#define M1_A    32
#define M1_B    33
#define M1_C    25
#define M1_EN   22

#define M2_PP   7
#define M2_A    26
#define M2_B    27
#define M2_C    14
#define M2_EN   12

#define DRIVER_V_LIMIT 6
#define MOT_A_LIMIT 1.5
#define MOT_V_LIMIT 1.0
#define MOT_V_ALIGN_LIMIT 0.9
// #define MOT_TORQUE_CTRL_TYPE TorqueControlType::estimated_current
// #define MOT_TORQUE_CTRL_TYPE TorqueControlType::voltage

#define MOT_CURRENT_CONTROL 1
#define MOT_VOLTAGE_CONTROL 0

#define MOT_CONTROL_TYPE MOT_CURRENT_CONTROL
#define MOT_KV  360.0
#define MOT_R   0.27

// =====================================
// SAFETY LIMITS
// =====================================
#define MOT_LIMIT          1.0   // Motor target limit

#define STALL_TORQUE_MIN   0.3   // Min target (A / V) leading to movement
#define STALL_VEL_MAX      1.0   // Max vel. considered as no movement
#define STALL_TIMEOUT_MS   300   // Timeout (in ms) of stall before throwing error

#define MAX_DESYNC_VEL_RAD 20.0  // Max vel difference between wheels
#define MAX_OPPOSITE_VEL_RAD 15.0  // 
#define SPIN_TIMEOUT_MS      200   // 
