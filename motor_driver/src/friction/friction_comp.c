/*
    friction_comp.c

    Drive friction compensation, see friction_comp.h.
*/
#include "friction_comp.h"

#include <math.h>

float frictionComp(const FrictionComp_t *fc, float i_cmd, float wheel_vel)
{
    if (fabsf(wheel_vel) >= fc->w_stuck) {
        // wheel turns: part of the Coulomb friction in the direction of motion
        return fc->k_kinetic * fc->i_coulomb * tanhf(wheel_vel / fc->w_smooth);
    }
    // wheel stuck: help it break away in the commanded direction, ramped in above a small dead zone
    float ramp = (fabsf(i_cmd) - fc->i_dead) / fc->i_ramp;
    if (ramp <= 0.0f) return 0.0f;
    if (ramp > 1.0f) ramp = 1.0f;
    return copysignf(fc->i_breakaway * ramp, i_cmd);
}
