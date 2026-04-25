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

// 
void IRAM_ATTR doA1(){ encoder1.handleA(); }
void IRAM_ATTR doB1(){ encoder1.handleB(); }

void IRAM_ATTR doA2(){ encoder2.handleA(); }
void IRAM_ATTR doB2(){ encoder2.handleB(); }

float mot1_target = 0.0;
float mot2_target = 0.0;

void set_pins_low_setup() {
  pinMode(22, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(22, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(32, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(32, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(33, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(33, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(25, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(25, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)

  pinMode(12, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(12, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(26, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(26, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(27, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(27, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
  pinMode(14, OUTPUT);     // Pin EN dla Drivera 2
  digitalWrite(14, LOW);   // Twarde wyłączenie Drivera 2 (przed startem)
}

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

  motor1.LPF_velocity.Tf = MOT_VEL_FILTER; 
  motor2.LPF_velocity.Tf = MOT_VEL_FILTER;

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
    target1 = constrain(target1, -MOT_LIMIT, MOT_LIMIT);
    target2 = constrain(target2, -MOT_LIMIT, MOT_LIMIT);
    
    motor1.move(target1);
    motor2.move(target2);
}

void motors_stop(){
    motor1.move(0.0);
    motor2.move(0.0);
}


float motors_synchronize(float target1, float target2) {
    static float target_angle_diff = 0.0;
    static bool is_turning = true;
    float correction = 0.0;

    if (abs(target1 - target2) < 0.05) {
        if (is_turning) {
            target_angle_diff = motor1.shaft_angle - motor2.shaft_angle;
            is_turning = false;
        }

        float current_diff = motor1.shaft_angle - motor2.shaft_angle;
        float angle_error = current_diff - target_angle_diff;

        correction = angle_error * MOT_K_SYNC_WHEELS;
        correction = constrain(correction, -0.5, 0.5);

    } else {
        is_turning = true;
    }
    
    return correction;
}

void motors_sync_move(float target1, float target2, bool enable_sync) {
    float m1_out = target1;
    float m2_out = target2;

    if ( enable_sync ) {
        float sync_corr = motors_synchronize(target1, target2);
        m1_out -= sync_corr;
        m2_out += sync_corr;
    }

    m1_out = constrain(m1_out, -MOT_LIMIT, MOT_LIMIT);
    m2_out = constrain(m2_out, -MOT_LIMIT, MOT_LIMIT);

    motors_move(m1_out, m2_out);
}


void check_motors_health(float target1, float target2, bool is_working_flag) {
    static uint32_t stall_timer = millis();
    static uint32_t spin_error_timer = millis();
    static uint32_t desync_timer = millis(); // 

    if ( !is_working_flag ) {
        stall_timer = millis();
        spin_error_timer = millis();
        desync_timer = millis(); // 
        return; 
    }

    float v1 = motor1.shaft_velocity;
    float v2 = motor2.shaft_velocity;

    // --------------------------------------------------------
    // 1. Stall Detection
    // --------------------------------------------------------
    bool is_pushing_hard = (abs(target1) > STALL_TORQUE_MIN) || (abs(target2) > STALL_TORQUE_MIN);  // are motors supposed to run?
    bool is_not_moving = (abs(v1) < STALL_VEL_MAX) || (abs(v2) < STALL_VEL_MAX);                            // are motors stationary when supposed to move?

    if (is_pushing_hard && is_not_moving) {
        if (millis() - stall_timer > STALL_TIMEOUT_MS) {
            sys_error = true;
            error_state = ERR_MOTOR_STALL;
        }
    } else {
        stall_timer = millis(); // Timer reset (if everything is ok)
    }

    // --------------------------------------------------------
    // 2. Desync Protection
    // --------------------------------------------------------
    float vel_difference = abs(v1 - v2);
    if (vel_difference > MAX_DESYNC_VEL_RAD) {
        
        if (millis() - desync_timer > DESYNC_TIMEOUT) { 
            sys_error = true;
            error_state = ERR_DESYNC;
        }
    } else {
        desync_timer = millis();
    }

    // --------------------------------------------------------
    // 3. Counter-Rotation / Yaw Spin
    // --------------------------------------------------------
    if ((v1 * v2 < 0) && (abs(v1) > MAX_OPPOSITE_VEL_RAD) && (abs(v2) > MAX_OPPOSITE_VEL_RAD)) {
        if (millis() - spin_error_timer > SPIN_TIMEOUT_MS) {
            sys_error = true;
            error_state = ERR_OPPOSITE_SPIN;
        }
    } else {
        spin_error_timer = millis(); // Timer reset (if everything is ok)
    }
}


void work() {
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
