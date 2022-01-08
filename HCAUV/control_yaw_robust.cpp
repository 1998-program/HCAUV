#include "HC.h"

bool HC::yaw_robust_init()
{
    pos_control.set_alt_target(0);
    last_roll = 0;
    last_pitch = 0;

    last_yaw = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();
    return true;
}

void HC::yaw_robust_run()
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
    //communicate with rasp
    //hc_yaw_robust_error
    // TARGET_YAW
    hc_yaw_robust_error = g.target_yaw - ahrs.yaw_sensor;

    send_to_rasp(hc_yaw_robust_error,control_mode_yaw_robust,hc_arm);
    


}