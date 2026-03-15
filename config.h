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
  ERR_COM_TIMEOUT
} error_states_t;

extern error_states_t error_state;
extern bool sys_error;

// =====================================
// COMMUNICATION
// =====================================
#define RX_PIN 16  // ESP32 RX Pin 
#define TX_PIN 17  // ESP32 TX Pin
#define UART_NR 2


// =====================================
// ENCODERS
// =====================================
#define ENC1_A  18
#define ENC1_B  19
#define ENC1_PPR    1000

// =====================================
// MOTORS
// =====================================
#define M1_PP   7
#define M1_A    26
#define M1_B    27
#define M1_C    14
#define M1_EN   12

#define M2_PP   7
#define M2_A    32
#define M2_B    33
#define M2_C    25
#define M2_EN   22

// =====================================
// GENERAL
// =====================================
#define MOT_LIMIT   1
