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
#define COMM_BAUD 115200 

#define FRAME_SIZE 13

// Data frame (1 + 1 + 2 + 4 + 4 + 1 = 13 bytes)
typedef struct __attribute__((packed)) {
    uint8_t startByte;   // 0xAA
    uint8_t command;     // CMD_XX
    uint16_t seq;        // +1 for every command send from master
    float   value1;      // 
    float   value2;      // 
    uint8_t checksum;    // XOR 
} MotorPacket_t;
static_assert(sizeof(MotorPacket_t) == FRAME_SIZE, "MotorPacket_t size mismatch with STM32");

#define FEEDBACK_FRAME_SIZE 32

// Feedback frame (1 + 1 + 1 + 2 + 2 + 6*4 + 1 = 32 bytes)
typedef struct __attribute__((packed)) {
    uint8_t   startByte; // 0xBB
    uint8_t   status;    // CMD_XX
    uint8_t   error;     // from error_states_t 
    uint16_t  rx_seq;    // seq from last processed command
    uint16_t  tx_seq;    // +1 for every feedback frame
    float mot1_vel;
    float mot1_pos;
    float mot1_A;
    float mot2_vel;
    float mot2_pos;
    float mot2_A;
    uint8_t checksum;    // XOR
} FeedbackPacket_t;
static_assert(sizeof(FeedbackPacket_t) == FEEDBACK_FRAME_SIZE, "FeedbackPacket_t size mismatch with STM32");

typedef enum {
    CMD_HELLO_MOTOR  = 0xFA, // HELLO FROM MOTOR
    CMD_HELLO_MASTER = 0xAF, // HELLO FROM MASTER
    CMD_START_INIT   = 0xDD, // 
    CMD_INIT         = 0x01,
    CMD_START        = 0x10,
    CMD_SET_VAL      = 0x11, // setting speed
    CMD_STOP         = 0x20, // 0x20,
    CMD_CLR_FLT      = 0x50, // clear errors
    CMD_RESET        = 0x51, // Reset     
    CMD_ERROR        = 0x60  // Error
} CommandType_t;

//////////////////////
extern MotorPacket_t rx_data;               // data recieved from master
extern volatile bool rx_msg_received;       // message recieved (from master) flag 
extern uint32_t last_valid_msg_time;        //
extern volatile bool connection_timer_flag; //
extern bool comm_timeout;                   //

//////////  Commander ////////////

extern Commander command;
extern bool user_start_trigger;      //
extern bool foc_initialized;         //
extern bool motor_test_enabled_flag; 

void doTarget(char* cmd);
void doInitMotors(char* cmd);
void doToggleDebug(char* cmd);
void doToggleTest(char* cmd);
void doStop(char* cmd);

//////////////////////
void comm_init();
uint8_t calculate_checksum(void* data, size_t length);
void process_commands();
void send_feedback(
    uint8_t status_cmd, uint8_t error,
    float mot1_v, float mot1_p, float mot1_A, 
    float mot2_v, float mot2_p, float mot2_A  );
// send status / error with no motor data
inline void send_status(uint8_t status_cmd, uint8_t error) { 
    send_feedback(status_cmd, error, 0, 0, 0, 0, 0, 0);
}

void connection_timer();
void telemetry();
