/*
    comm.h

*/

#pragma once
#include <Arduino.h>
#include "config.h"
#include <stdint.h>

/*
  STM PA2 -> TX   ESP -> 16 (RX) 
  STM PA3 -> RX   ESP -> 17 (TX) (czarne)
*/

#define FRAME_SIZE 11

// Data frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    float   value1;       // 
    float   value2;       // 
    uint8_t checksum;    // XOR 
} MotorPacket_t;

typedef enum {
    CMD_HELLO_MOTOR  = 0xFA, // HELLO FROM MOTOR
    CMD_HELLO_MASTER = 0xAF, // HELLO FROM MASTER
    CMD_INIT    = 0xDD,
    CMD_START   = 0x10,
    CMD_SET_VAL = 0x11, // Ustawienie prędkości
    CMD_STOP    = 0x55, // 0x20,
    // CMD_SET_TRQ = 0x40, 
    CMD_CLR_FLT = 0x50  // Kasowanie błędów
} CommandType_t;

// Feedback frame (1 + 1 + 4 + 4 + 1 = 11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte;    // 0xBB
    uint8_t   status;       //
    float value1;
    float value2;
    uint8_t   checksum;     // XOR
} FeedbackPacket_t; // feedback to master

//////////////////////
extern HardwareSerial SerialCTRL;
extern MotorPacket_t rx_data;
extern FeedbackPacket_t tx_data;
extern volatile bool rx_msg_received;
extern uint32_t last_valid_msg_time;
extern uint32_t last_connection_time;
extern volatile bool connection_timer_flag;

extern bool comm_timeout;

//////////  Commander ////////////

extern Commander command;
extern bool user_start_trigger;
extern bool foc_initialized;
extern bool debug_enabled;
extern bool motor_test_enabled_flag;
extern uint32_t last_telemetry_time;

void doMotors(char* cmd);
void doInitMotors(char* cmd);
void doToggleDebug(char* cmd);
void doToggleTest(char* cmd);

//////////////////////
void comm_init();
uint8_t calculate_checksum(void* data, size_t length);
void process_commands();
const char* getErrorText(error_states_t state);
void send_feedback(uint8_t status_cmd, float mot1_v, float mot2_v);
void connection_timer();
