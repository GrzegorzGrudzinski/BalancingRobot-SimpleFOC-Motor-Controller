#include <SimpleFOC.h>
#include "config.h"
#include "comm.h"
#include "motor.h"

typedef enum {
  CONNECT,
  INIT,
  WORK,
  ERROR
} robot_states_t;

robot_states_t state = CONNECT;

float mot1_target = 0.0;
float mot2_target = 0.0; // Na przyszłość, gdy odkomentujesz drugi silnik

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor1, cmd); }

bool debug_enabled = true;
void doToggleDebug(char* cmd) {
  debug_enabled = !debug_enabled;
  Serial.println(debug_enabled ? "Debug ON" : "Debug OFF");
}
uint32_t last_telemetry_time = 0;

// =====================================
// 
// =====================================

void setup() { 
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
 
  command.add('M', doMotor, "Motor");
  command.add('D', doToggleDebug, "Toggle Debug Telemetry");
  
  comm_init();
  motors_setup();

  Serial.println(F("Ready"));
  _delay(1000);
  last_valid_msg_time = millis();
}

void loop() {
  // main FOC algorithm function
  motors_loop_task();
  motors_move(mot1_target, 0.0);
  process_commands();
  command.run();
  
  switch (state) {
    case CONNECT:
    /*
      wait for connection with master
    */
      mot1_target = 0.0;
      mot2_target = 0.0;

      if (rx_data.command == CMD_INIT){
        state = INIT;
      }
      break;
    case INIT:
    /*
      wait until every component is ready
      (implement timeout)
    */
      if (rx_data.command == CMD_START)
        state = WORK;
      break;
    case WORK:
    /*
      normal operation
    */
      work();
      break;
    case ERROR:
    /*
      error state - 
    */
      mot1_target = 0.0;
      mot2_target = 0.0;
      break;
  }
  telemetry();

}

void work () {
    if (rx_msg_received) {
        last_valid_msg_time = millis();
        comm_timeout = false;

        switch(rx_data.command) {
            case CMD_HELLO_MASTER :
                //
                break;
            case CMD_SET_VAL:
                mot1_target = rx_data.value1;
                mot2_target = rx_data.value2;

                if (error_state != ERR_INIT) {
                    sys_error = false;
                    error_state = ERR_OK;
                }
                break;
            case CMD_STOP: // STOP (Upadek)
                mot1_target = 0.0;
                mot2_target = 0.0;
                break;
            default:
                mot1_target = 0.0;
                mot2_target = 0.0;
                sys_error = true;
                error_state = ERR_INV_COMMAND;
                // comm_timeout = true;
                break;
        }

        rx_msg_received = false; // wait for the next command
    }

    if (millis() - last_valid_msg_time > COMM_TIMEOUT_MS) {
        if (error_state != ERR_INIT) {
            mot1_target = 0.0;
            mot2_target = 0.0;
            sys_error = true;
            error_state = ERR_COM_TIMEOUT;
            if (!comm_timeout) {
                comm_timeout = true;
                // if (debug_enabled) Serial.println("BLAD: Timeout UART! Zatrzymuje silniki.");
            }
        }
    }

    // Motion control function
    if (isnan(mot1_target) || isnan(mot2_target)) {
        mot1_target = 0.0;
        mot2_target = 0.0;
        sys_error = true;
        error_state = ERR_IS_NAN;
    }

    mot1_target = constrain(mot1_target, -MOT_LIMIT, MOT_LIMIT);
    mot2_target = constrain(mot2_target, -MOT_LIMIT, MOT_LIMIT);
}

void telemetry() {
    if (debug_enabled) {
        if (millis() - last_telemetry_time > 50) { // 20 Hz
            last_telemetry_time = millis();
            if (!sys_error) {
                Serial.print("Mot 1 target: ");
                Serial.print(mot1_target);
                Serial.print("\tMot1 velocity: ");
                Serial.println(motor1.shaft_velocity);

                // Serial.print("Mot 2 target: ");
                // Serial.print(mot2_target);
                // Serial.print("\tMot2 angle: ");
                // Serial.println(motor2.shaft_angle);
            }
            else {
                Serial.print("ERROR\t");
                Serial.print(getErrorText(error_state));
                Serial.print("\tRX Command: ");
                Serial.print(rx_data.command, HEX);
                Serial.print(" ");
                Serial.print(rx_data.value1);
                Serial.print("\tTarget");
                Serial.println(mot1_target);
            }
        }
    }
}
