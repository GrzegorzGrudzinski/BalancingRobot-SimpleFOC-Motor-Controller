/*
  motor_driver.ino
  (main)
*/

#include <SimpleFOC.h>
#include "config.h"
#include "comm.h"
#include "motor.h"

typedef enum {
  STANDBY,
  CONNECT,
  INIT,
  MOTOR_TEST,
  WORK,
  ERROR
} robot_states_t; 

robot_states_t state = STANDBY;

void LookForErrors();

float mot1_target = 0.0;
float mot2_target = 0.0; // Na przyszłość, gdy odkomentujesz drugi silnik

// =====================================
// 
// =====================================

void setup() { 
  set_pins_low_setup();

  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
 
  command.add('T', doMotors, "Target current");
  command.add('D', doToggleDebug, "Toggle Debug Telemetry");
  command.add('S', doToggleTest, "Toggle Motor Test Mode"); //
  command.add('I', doInitMotors, "Init/Start FOC"); 

  comm_init();
  // motors_setup();

  Serial.println(F("Ready - waiting for 'S' command "));
  _delay(1000);
  last_valid_msg_time = millis();
}

void loop() {
  process_commands();
  command.run();

  // main FOC algorithm function
  if (foc_initialized) {
    motors_loop_task();
    bool sync_enabled_flag = (state == WORK && error_state == ERR_OK);
    motors_sync_move(mot1_target, mot2_target, sync_enabled_flag);
  }

  switch (state) {
  case STANDBY:
    if ( user_start_trigger) {
      Serial.println("Starting FOC calibration... ");

      motors_setup();

      if (sys_error) {
        state = ERROR;
      } 
      else {
        motor1.target = 0.0;
        motor2.target = 0.0;

        Serial.println("Stabilizing filters...");

        motor1.disable(); 
        motor2.disable();
        for(int i = 0; i < 100; i++) {
            motor1.sensor->update();
            motor1.shaft_velocity = motor1.LPF_velocity(motor1.sensor->getVelocity());
            
            motor2.sensor->update();
            motor2.shaft_velocity = motor2.LPF_velocity(motor2.sensor->getVelocity());
            
            _delay(2); // 
        }
        
        motor1.PID_velocity.reset(); 
        motor2.PID_velocity.reset();
        
        // motor1.enable();
        // motor2.enable();

        foc_initialized = true; 
        state = CONNECT;
        last_valid_msg_time = millis();
        Serial.println("Motors ready. Waiting for communication init");
      }
      user_start_trigger = false; 
    }
    break;

  case CONNECT: // wait for connection with master
    mot1_target = 0.0;
    mot2_target = 0.0;
    connection_timer();
    if (rx_data.command == CMD_INIT){
      state = INIT;
      
      Serial.println("Connection successful - waiting for START command");
    }
    break;

  case INIT: // wait until every component is ready  (implement timeout)
    mot1_target = 0.0;
    mot2_target = 0.0;
    if (rx_data.command == CMD_INIT) {
      if (motor_test_enabled_flag) {
        state = MOTOR_TEST;
      }
      else {
        state = WORK;
      }

      last_valid_msg_time = millis(); // reset timer while master is getting ready

      motor1.enable();
      motor2.enable();

      Serial.println("Ready - starting to balance");
    }
    break;
  case MOTOR_TEST:
  
    break;
  case WORK: // normal operation
    work();
    break;

  case ERROR: // error state
    mot1_target = 0.0;
    mot2_target = 0.0;
    
    motor1.disable();
    motor2.disable();
    
    break;
  }

  if (state != STANDBY) {
    if (rx_msg_received || (state == CONNECT && connection_timer_flag)) {
    uint8_t status_to_send;
    switch (state) {
      case CONNECT: status_to_send = CMD_HELLO_MOTOR; break;
      case INIT:    status_to_send = CMD_START;       break;
      case WORK:    status_to_send = CMD_SET_VAL;     break; 
      default:      status_to_send = CMD_STOP;        break;
    }

    // Send status to master
    send_feedback(status_to_send, mot1_target, mot2_target); 
    
    rx_msg_received = false; // wait for the next command
    connection_timer_flag = false;
    }
    telemetry();
  }
  
  // DEBUG - MOVE UP!!!
  LookForErrors();
}

void LookForErrors() {
  if ( !sys_error ) {
      bool is_working_flag = (state == WORK);
      check_motors_health(mot1_target, mot2_target, is_working_flag);
  }

  if (sys_error || (error_state != ERR_OK) || comm_timeout) {
    state = ERROR;
  }
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
                mot1_target = - rx_data.value1;
                mot2_target = - rx_data.value2;

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
    }

    if (millis() - last_valid_msg_time > COMM_TIMEOUT_MS) {
        if (error_state != ERR_INIT) {
            mot1_target = 0.0;
            mot2_target = 0.0;
            sys_error = true;
            error_state = ERR_COM_TIMEOUT;
            if (!comm_timeout) {
                comm_timeout = true;
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
