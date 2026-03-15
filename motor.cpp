#include "motor.h"

BLDCMotor motor1 = BLDCMotor(M1_PP);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_A, M1_B, M1_C, M1_EN);
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
  driver1.voltage_limit = 6;
  if(!driver1.init()){
    Serial.println("Driver init failed!");
    sys_error = true;
    error_state = ERR_INIT;
    return;
  }
  motor1.linkDriver(&driver1);
  
  // motor1.phase_resistance = 0.27f; 
  // motor1.KV_rating = 360.0f;       

  motor1.voltage_limit = 1.0;
  motor1.voltage_sensor_align = 1.0; 
  motor1.target = 0.0;

  motor1.torque_controller = TorqueControlType::voltage;
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