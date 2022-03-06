#include "HC.h"

bool HC::att_robust_init()
{
    pos_control.set_alt_target(0);
    if (prev_control_mode == DEPTH_HOLD_PID) {
        last_roll = ahrs.roll_sensor;
        last_pitch = ahrs.pitch_sensor;
    } else {
        last_roll = 0;
        last_pitch = 0;
    }
    last_yaw = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();
    return true;
}

void HC::att_robust_run()
{    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_roll = 0;
        last_pitch = 0;
        last_yaw = ahrs.yaw_sensor;
        return;
    }
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
//  hc_att_robust_error
//    handle_attitude();

    // output pilot's throttle
//    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    // attitude_control.hc_input_euler_angle_roll_pitch_yaw(g.target_roll * 100, g.target_pitch * 100 , g.target_yaw * 100, true);
}