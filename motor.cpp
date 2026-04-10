/*
  motor.cpp
*/

#include "motor.h"

BLDCMotor motor1 = BLDCMotor(M1_PP);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_A, M1_B, M1_C, M1_EN);
Encoder encoder1 = Encoder(ENC1_A, ENC1_B, ENC1_PPR);

BLDCMotor motor2 = BLDCMotor(M2_PP);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_A, M2_B, M2_C, M2_EN);
Encoder encoder2 = Encoder(ENC2_A, ENC2_B, ENC2_PPR);

// IRAM_ATTR ????
void IRAM_ATTR doA1(){ encoder1.handleA(); }
void IRAM_ATTR doB1(){ encoder1.handleB(); }

void IRAM_ATTR doA2(){ encoder2.handleA(); }
void IRAM_ATTR doB2(){ encoder2.handleB(); }

void motors_setup() {

  // initialize encoder sensor hardware
  // encoder1.quadrature = Quadrature::ON;
  encoder1.init();
  encoder1.enableInterrupts(doA1, doB1); 
  motor1.linkSensor(&encoder1);
  
  // encoder2.quadrature = Quadrature::ON;
  encoder2.init();
  encoder2.enableInterrupts(doA2, doB2); 
  motor2.linkSensor(&encoder2);

  // driver init
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = DRIVER_V_LIMIT;
  if(!driver1.init()){
    Serial.println("Driver 1 init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
  motor1.linkDriver(&driver1);

  driver2.voltage_power_supply = 12;
  driver2.voltage_limit = DRIVER_V_LIMIT;
  if(!driver2.init()){
    Serial.println("Driver 2 init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
  motor2.linkDriver(&driver2);
  
  motor1.voltage_sensor_align = MOT_V_ALIGN_LIMIT; 
  motor1.target = 0.0;

  motor2.voltage_sensor_align = MOT_V_ALIGN_LIMIT; 
  motor2.target = 0.0;

  #if MOT_CONTROL_TYPE // 1 -> current , 0 -> voltage
    motor1.phase_resistance = MOT_R; 
    motor1.KV_rating = MOT_KV;       
    motor1.current_limit = MOT_A_LIMIT;

    motor2.phase_resistance = MOT_R; 
    motor2.KV_rating = MOT_KV;       
    motor2.current_limit = MOT_A_LIMIT;

    motor1.torque_controller = TorqueControlType::estimated_current;
    motor1.controller = MotionControlType::torque;
    
    motor2.torque_controller = TorqueControlType::estimated_current;
    motor2.controller = MotionControlType::torque;
  #else 
    motor1.voltage_limit = MOT_V_LIMIT;
    motor2.voltage_limit = MOT_V_LIMIT;

    motor1.torque_controller = TorqueControlType::voltage;
    motor1.controller = MotionControlType::torque;
    
    motor2.torque_controller = TorqueControlType::voltage;
    motor2.controller = MotionControlType::torque;
  #endif
  // motor1.torque_controller = MOT_TORQUE_CTRL_TYPE;
  // motor1.controller = MotionControlType::torque;
  
  // motor2.torque_controller = MOT_TORQUE_CTRL_TYPE;
  // motor2.controller = MotionControlType::torque;

  if(!motor1.init() || !motor1.initFOC()){
    Serial.println("Motor/FOC init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }

  if(!motor2.init() || !motor2.initFOC()){
    Serial.println("Motor/FOC init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
}

void motors_loop_task() {
    motor1.loopFOC();
    motor2.loopFOC();
}

void motors_move(float target1, float target2) {
    motor1.move(target1);
    motor2.move(target2);
    
}

void motors_stop(){
    motor1.move(0.0);
    motor2.move(0.0);
}