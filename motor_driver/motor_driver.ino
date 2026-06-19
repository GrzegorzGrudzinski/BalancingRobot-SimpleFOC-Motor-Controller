/*
  motor_driver.ino
  (main)
*/

#include <SimpleFOC.h>
#include "config.h"
#include "comm.h"
#include "motor.h"

/*
  command.add('I', doInitMotors, "Init/Start FOC"); 
  command.add('X', doStop, "Stop motors"); 
  command.add('D', doToggleDebug, "Toggle Debug Telemetry");
  command.add('S', doToggleTest, "Toggle Motor Test Mode"); //
  command.add('T', doTarget, "Target current"); (After entering test mode)

*/

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
      bool sync_enabled_flag = (state == RUN && error_state == ERR_OK);
      motors_sync_move(mot1_target, mot2_target, sync_enabled_flag);
    }
  }

  // 
  switch (state) {
  case STANDBY:
    motors_disable();

    if (rx_data.command == CMD_START_INIT) {
      user_start_trigger = true;
      
      send_feedback( CMD_START_INIT, 0.0, 0.0, 0.0, 0.0); 
    }

    if ( user_start_trigger ) {
      Serial.println("Starting FOC calibration... ");

      motors_setup();

      if (sys_error) {
        state = ERROR;
      } 
      else {
        motors_disable();

        Serial.println("Stabilizing filters...");
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
        state = RUN;
      }

      last_valid_msg_time = millis(); // reset timer while master is getting ready

      motor1.enable();
      motor2.enable();

      Serial.println("Ready - starting to balance");
    }
    break;
  case MOTOR_TEST:
    if (!motor_test_enabled_flag) {
      mot1_target = 0.0;
      mot2_target = 0.0;
      state = RUN;
      Serial.println("Motor Testing disabled. Back to RUN");
    }
    break;

  case RUN: // normal operation
    if (motor_test_enabled_flag) {
      mot1_target = 0.0;
      mot2_target = 0.0;
      state = MOTOR_TEST;
      Serial.println("MOTOR_TEST enabled. Use T command to set target (eg. T0.5)");
    } else {
      work();

      if (error_state != ERR_OK) {
        state = ERROR;
      }
      else if (rx_data.command == STOP) {
        state = STOP;
      }
    }
    break;

  case STOP: // 
    motors_disable();
    
    state = STANDBY;

    break;
    
  case ERROR: // error state
    motors_disable();
    
    if (rx_data.command == CMD_CLR_FLT) {
      last_valid_msg_time = millis();
      sys_error = false;
      error_state = ERR_OK;

      send_feedback(CMD_CLR_FLT, 0.0, 0.0, 0.0, 0.0);
      
      state = STANDBY;
    }

    break;
  }

  // Feedback 
  if (state != (STANDBY) ) {
    if (rx_msg_received || (state == CONNECT && connection_timer_flag)) {
    uint8_t status_to_send;
    switch (state) {
      case CONNECT: status_to_send = CMD_HELLO_MOTOR; break;
      case INIT:    status_to_send = CMD_START;       break;
      case RUN:     status_to_send = CMD_SET_VAL;     break; 
      case STOP:    status_to_send = CMD_STOP;        break;
      case ERROR:   status_to_send = CMD_ERROR;       break;
      default:      status_to_send = CMD_STOP;        break;
    }

    // Send status to master
    send_feedback(status_to_send, 
                  motor1.shaft_velocity, motor2.shaft_velocity,
                  motor1.shaft_angle,    motor2.shaft_angle
                 ); 
    
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
      bool is_working_flag = (state == RUN);
      check_motors_health(mot1_target, mot2_target, is_working_flag);
  }

  if (sys_error || (error_state != ERR_OK) || comm_timeout) {
    state = ERROR;
  }
}
