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

void motors_setup();
void motors_loop_task();
void motors_move(float target1, float target2);
void motors_stop();