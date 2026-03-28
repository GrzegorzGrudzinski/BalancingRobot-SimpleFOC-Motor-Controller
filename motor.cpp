/*
  motor.cpp
*/

#include "motor.h"

BLDCMotor motor1 = BLDCMotor(M2_PP);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M2_A, M2_B, M2_C, M2_EN);
Encoder encoder1 = Encoder(ENC1_A, ENC1_B, ENC1_PPR);

// IRAM_ATTR ????
void doA(){ encoder1.handleA(); }
void doB(){ encoder1.handleB(); }

void motors_setup() {

  // initialize encoder sensor hardware
  encoder1.quadrature = Quadrature::ON;
  // encoder1.pullup = Pullup::USE_EXTERN;

  encoder1.init();
  encoder1.enableInterrupts(doA, doB); 
  motor1.linkSensor(&encoder1);

  // driver init
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = DRIVER_V_LIMIT;
  if(!driver1.init()){
    Serial.println("Driver init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
  motor1.linkDriver(&driver1);
  
  #if MOT_PARAMETERS_ON // 0
    motor1.phase_resistance = MOT_R; 
    motor1.KV_rating = MOT_KV;       
  #endif
  motor1.voltage_limit = MOT_V_LIMIT;
  motor1.voltage_sensor_align = MOT_V_ALIGN_LIMIT; 
  motor1.target = 0.0;

  motor1.torque_controller = MOT_TORQUE_CTRL_TYPE;
  motor1.controller = MotionControlType::torque;

  if(!motor1.init() || !motor1.initFOC()){
    Serial.println("Motor/FOC init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
}

void motors_loop_task() {
    motor1.loopFOC();
    // motor2.loopFOC();
}

void motors_move(float target1, float target2) {
    motor1.move(target1);
    // motor2.move(target2);
    
}

void motors_stop(){
    motor1.move(0.0);
    // motor2.move(0.0);
}