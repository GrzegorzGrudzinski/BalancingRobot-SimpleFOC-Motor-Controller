/*
    comm.h

*/

#pragma once
#include <Arduino.h>
#include "config.h"

#define COMM_TIMEOUT_MS 500 // 0.5 sec

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

// Feedback frame (1 + 1 + 4 + 1 = 7 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte;    // 0xBB
    uint8_t   status;       //
    float     value;    // 
    uint8_t   checksum;     // XOR
} FeedbackPacket_t;

//////////////////////
extern HardwareSerial SerialCTRL;
extern MotorPacket_t rx_data;
extern volatile bool rx_msg_received;
extern uint32_t last_valid_msg_time;

extern bool comm_timeout;


//////////////////////
void comm_init();
uint8_t calculate_checksum(void* data, size_t length);
void process_commands();
const char* getErrorText(error_states_t state);
