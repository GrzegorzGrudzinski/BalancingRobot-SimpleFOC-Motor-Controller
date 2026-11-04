/*
    friction_comp.h

    Drive friction compensation: current added to the command of one wheel.
*/
#ifndef FRICTION_COMP_H_
#define FRICTION_COMP_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float i_coulomb;    // [A] friction while the wheel turns
    float k_kinetic;    // [-] share of it compensated while turning (< 1: no negative damping)
    float w_smooth;     // [rad/s] tanh width around zero speed
    float w_stuck;      // [rad/s] slower wheel counts as stuck
    float i_breakaway;  // [A] help when stuck; keep below the smallest break-away current
    float i_dead;       // [A] no help for smaller commands (no chatter around zero)
    float i_ramp;       // [A] help grows from 0 to full over this command range
} FrictionComp_t;

/* Current to add to i_cmd [A]. i_cmd [A] and wheel_vel [rad/s, relative to the body]
   must use the same sign convention (positive current turns the wheel forward).
   motors_sync_move() passes the STM command (without the sync correction) and the speed of the same wheel. */
float frictionComp(const FrictionComp_t *fc, float i_cmd, float wheel_vel);

#ifdef __cplusplus
}
#endif

#endif /* FRICTION_COMP_H_ */
