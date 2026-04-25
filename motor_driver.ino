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

// =====================================
// 
// =====================================

void setup() { 
  set_pins_low_setup();

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

    if (state == MOTOR_TEST) {
      motors_sync_move(mot1_target, mot2_target, false);
    } else {
      bool sync_enabled_flag = (state == WORK && error_state == ERR_OK);
      motors_sync_move(mot1_target, mot2_target, sync_enabled_flag);
    }
  }

  // 
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

  // Feedback 
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
