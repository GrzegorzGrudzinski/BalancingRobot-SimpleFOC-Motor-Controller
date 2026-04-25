/*
    motor.h

*/

#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "comm.h" // for sys_error flag

extern BLDCMotor motor1;
extern BLDCMotor motor2;

extern float mot1_target;
extern float mot2_target; 

void set_pins_low_setup();
void motors_setup();
void motors_loop_task();
void motors_move(float target1, float target2);
void motors_stop();
float motors_synchronize();
void motors_sync_move(float target1, float target2, bool enable_sync);
void check_motors_health(float target1, float target2, bool is_working_flag);
void work();