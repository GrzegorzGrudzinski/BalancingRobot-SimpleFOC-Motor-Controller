#include "esp32-hal.h"
/*
    comm.cpp
*/

#include <Arduino.h>
#include <SimpleFOC.h> // for commander interface
#include "comm.h"
#include "motor.h" // for motor object

HardwareSerial SerialCTRL(UART_NR); 

MotorPacket_t rx_data = {0}; // 
FeedbackPacket_t tx_data = {0};

uint8_t rx_buffer[FRAME_SIZE]; 
volatile bool rx_msg_received = false; 

uint32_t last_valid_msg_time = 0;
bool comm_timeout = false;
uint32_t last_connection_time = 0;
volatile bool connection_timer_flag = false; 


error_states_t error_state = ERR_OK;
bool sys_error = false;


// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { 
    command.scalar(&mot1_target, cmd);
    command.scalar(&mot2_target, cmd);
}

// Init command
bool user_start_trigger = false;  // Init start flag (from serial)
bool foc_initialized = false;     // FOC initialized flag

void doInitMotors(char* cmd) { 
  user_start_trigger = true; 
}

// Debug command
bool debug_enabled = true;

void doToggleDebug(char* cmd) {
  debug_enabled = !debug_enabled;
  Serial.println(debug_enabled ? "Debug ON" : "Debug OFF");
}
uint32_t last_telemetry_time = 0;

// Motor test mode command
bool motor_test_enabled_flag = false;

void doToggleTest(char* cmd) {
  motor_test_enabled_flag = !motor_test_enabled_flag;
  Serial.println(motor_test_enabled_flag ? "Test Mode ON" : "Test Mode OFF");
}


//////////////////////////////

void comm_init() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
 
  command.add('T', doTarget, "Target current");
  command.add('D', doToggleDebug, "Toggle Debug Telemetry");
  command.add('S', doToggleTest, "Toggle Motor Test Mode"); //
  command.add('I', doInitMotors, "Init/Start FOC"); 

  // communication with master
  SerialCTRL.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}

void connection_timer() {
  if (millis() - last_connection_time > 50) {
      connection_timer_flag = true;
      last_connection_time = millis();
  }
}

uint8_t calculate_checksum(void* data, size_t length)
{
    if(length==0 ) return 0;

    uint8_t checksum = 0;
    uint8_t* byte = (uint8_t*)data;

    for (size_t i = 0; i < length-1; i++) {
        checksum ^= byte[i];
    }

    return checksum;
}

void process_commands()
{
    static uint8_t rx_state = 0; // 0 -looking for start byte, 1 - receiving data
    static uint8_t rx_index = 0;

    while (SerialCTRL.available() > 0) {
        uint8_t incoming_byte = SerialCTRL.read();
        // if (incoming_byte == 0xAA && rx_state == 1 && rx_index > 0 && rx_index < sizeof(MotorPacket_t)) {
        //     rx_index = 0;
        // }

        if (rx_state == 0) {
            if (incoming_byte == 0xAA) { // Start byte found
                rx_buffer[0] = incoming_byte;
                rx_index = 1;
                rx_state = 1;
            }
        }
        else if (rx_state == 1) {
            rx_buffer[rx_index] = incoming_byte;
            rx_index++;

            // Full packet received?
            if(rx_index >= FRAME_SIZE) {
                //calcuate checksum and verify
                uint8_t checksum = calculate_checksum(rx_buffer, FRAME_SIZE);
                if (rx_buffer[FRAME_SIZE-1] == checksum) {
                    // Valid packet
                    // memcpy(&rx_data, rx_buffer, FRAME_SIZE);

                    // MotorPacket_t* recieved_pkt = (MotorPacket_t*)rx_buffer;
                    // rx_data = *recieved_pkt;

                    rx_data.startByte = rx_buffer[0];
                    rx_data.command   = rx_buffer[1];
                    memcpy(&rx_data.value1, &rx_buffer[2], sizeof(float)); // float 1
                    memcpy(&rx_data.value2, &rx_buffer[6], sizeof(float)); // float 2
                    rx_data.checksum  = rx_buffer[10];

                    rx_msg_received = true;
                }
                else {
                    // error
                }
                rx_state = 0; // Reset state to look for next packet
            }
        }
    } // while
}

const char* getErrorText(error_states_t state)
{
    switch (state) {
        case ERR_OK:            return "OK";
        case ERR_INIT:          return "INIT_FAILED";
        case ERR_COMMUNICATION: return "COMMUNICATION_ERROR";
        case ERR_INV_COMMAND:   return "INVALID_COMMAND";
        case ERR_IS_NAN:        return "TARGET_IS_NAN";
        case ERR_COM_TIMEOUT:   return "TIMEOUT";
        case ERR_MOTOR_STALL:   return "MOTOR_STALLED";  
        case ERR_DESYNC:        return "MOTORS_DESYNC"; 
        case ERR_OPPOSITE_SPIN: return "OPPOSITE_SPIN"; 
        default:                return "UNKNOWN_ERROR";
    }
}


void send_feedback(uint8_t status_cmd, float mot1_v, float mot2_v) {
    tx_data.startByte = 0xBB;
    tx_data.status = status_cmd;
    tx_data.value1 = mot1_v;
    tx_data.value2 = mot2_v; 

    // Używamy &tx_data, a nie tx_packet
    tx_data.checksum = calculate_checksum((uint8_t*)&tx_data, sizeof(FeedbackPacket_t));

    SerialCTRL.write((uint8_t*)&tx_data, sizeof(FeedbackPacket_t));
}




void telemetry() {
    if (debug_enabled) {
        if (millis() - last_telemetry_time > 50) { // 20 Hz
            last_telemetry_time = millis();
            if (!sys_error) {
                Serial.print("Mot 1 target: ");
                Serial.print(mot1_target);
                Serial.print("\tMot1 voltage q: ");
                Serial.print(motor1.voltage.q);
                Serial.print("\tMot1 voltage d: ");
                Serial.print(motor1.voltage.d);
                Serial.print("\tMot1 velocity: ");
                Serial.print(motor1.shaft_velocity);
                Serial.print("\tMot1 angle: ");
                Serial.print(motor1.shaft_angle);

                Serial.print("\t\tMot 2 target: ");
                Serial.print(mot2_target);
                Serial.print("\tMot2 voltage q: ");
                Serial.print(motor2.voltage.q);
                Serial.print("\tMot2 voltage d: ");
                Serial.print(motor2.voltage.d);
                Serial.print("\tMot2 velocity: ");
                Serial.print(motor2.shaft_velocity);
                Serial.print("\tMot2 angle: ");
                Serial.println(motor2.shaft_angle);
            }
            else {
                Serial.print("ERROR\t");
                Serial.print(getErrorText(error_state));
                Serial.print("\tRX Command: ");
                Serial.print(rx_data.command, HEX);
                Serial.print(" ");
                Serial.print(rx_data.value1);
                Serial.print(" ");
                Serial.print(rx_data.value2);
                Serial.print("\tTarget 1 ");
                Serial.print(mot1_target);
                Serial.print("\tMot1 velocity: ");
                Serial.print(motor1.shaft_velocity);
                Serial.print("\tTarget 2 ");
                Serial.print(mot2_target);
                Serial.print("\tMot2 velocity: ");
                Serial.print(motor2.shaft_velocity);
                Serial.println("");
            }
        }
    }
}
