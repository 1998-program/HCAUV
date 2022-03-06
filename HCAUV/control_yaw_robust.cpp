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
        // attitude_control.set_throttle_out(0,true,g.throttle_filt);
        // attitude_control.relax_attitude_controllers();
        last_roll = 0;
        last_pitch = 0;
        hc_yaw_robust_error = 0;
        hc_yaw_robust_force = 0;
        last_yaw = ahrs.yaw_sensor;
        
        return;
    }
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    //communicate with rasp
    //hc_yaw_robust_error
    // TARGET_YAW
    if(g.dvl_on == 0){
        hc_yaw_robust_error = g.target_yaw * 100  - ahrs.yaw_sensor;
        if(hc_yaw_robust_error > 18000){
            hc_yaw_robust_error = hc_yaw_robust_error - 36000;
        }
        else if(hc_yaw_robust_error < -18000){
            hc_yaw_robust_error = hc_yaw_robust_error + 36000;
        }
        hc_yaw_robust_error = radians(hc_yaw_robust_error * 0.01);
        send_to_rasp(hc_yaw_robust_error,control_mode_yaw_robust,hc_arm);
    }
    else{
        hc_yaw_robust_error = g.target_yaw  - hc_dvl50_yaw;
        if(hc_yaw_robust_error > 180){
            hc_yaw_robust_error = hc_yaw_robust_error - 360;
        }
        else if(hc_yaw_robust_error < -180){
            hc_yaw_robust_error = hc_yaw_robust_error + 360;
        }
        hc_yaw_robust_error = radians(hc_yaw_robust_error);
        send_to_rasp(hc_yaw_robust_error,control_mode_yaw_robust,hc_arm);
    }
    hal.uartC->printf("hc_yaw_robust_error:%f\n",hc_yaw_robust_error);
    motors.set_throttle(channel_throttle->norm_input());


    


}