#include "HC.h"
/* initialization roll pitch yaw */
bool HC::yaw_PID_init() 
{
    // pos_control.set_alt_target(0);
    // if (prev_control_mode == DEPTH_HOLD_PID) {
    //     last_roll = ahrs.roll_sensor;
    //     last_pitch = ahrs.pitch_sensor;
    // } else {
    last_roll = 0;
    last_pitch = 0;
    // }
    last_yaw = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();
    return true;
}

void HC::yaw_PID_run()
{    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // attitude_control.set_throttle_out(0,true,g.throttle_filt);
        // attitude_control.relax_attitude_controllers();
        last_roll = 0;
        last_pitch = 0;
        last_yaw = ahrs.yaw_sensor;
        hc_yaw_pid_error = 0;
        hc_yaw_pid_force = 0;
        hal.uartC->printf("fxc\n");
        return;
    }
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//    handle_attitude();

    // output pilot's throttle
//    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
    // hal.uartC->printf("yaw_PID_run\n");
    // attitude_control.hc_input_euler_angle_roll_pitch_yaw(0, 0, g.target_yaw * 100, true);
    if(g.dvl_on == 0){
        hc_yaw_pid_error = g.target_yaw * 100  - ahrs.yaw_sensor;
        if(hc_yaw_pid_error > 18000){
            hc_yaw_pid_error = hc_yaw_pid_error - 36000;
        }
        else if(hc_yaw_pid_error < -18000){
            hc_yaw_pid_error = hc_yaw_pid_error + 36000;
        }
        hc_yaw_pid_error = radians(hc_yaw_pid_error * 0.01);
        hc_yaw_pid_force = attitude_control.HC_POS_PID(0,-hc_yaw_pid_error,false,45,0.05,35);
        hc_yaw_pid_force = constrain_float(hc_yaw_pid_force,-33.26,33.26);
        motors.set_yaw_force(hc_yaw_pid_force);
        // send_to_rasp(hc_yaw_robust_error,control_mode_yaw_robust,hc_arm);
    }
    else{
        hc_yaw_pid_error = g.target_yaw * 100  - hc_dvl50_yaw;
        if(hc_yaw_pid_error > 18000){
            hc_yaw_pid_error = hc_yaw_pid_error - 36000;
        }
        else if(hc_yaw_pid_error < -18000){
            hc_yaw_pid_error = hc_yaw_pid_error + 36000;
        }
        hc_yaw_pid_error = radians(hc_yaw_pid_error * 0.01);
        hc_yaw_pid_force = attitude_control.HC_POS_PID(0,-hc_yaw_pid_error,false,45,0.05,35);
        hc_yaw_pid_force = constrain_float(hc_yaw_pid_force,-33.26,33.26);
        motors.set_yaw_force(hc_yaw_pid_force);
        // send_to_rasp(hc_yaw_robust_error,control_mode_yaw_robust,hc_arm);
    }
    // hal.uartC->printf("hc_yaw_pid_error:%f\n",hc_yaw_pid_error);
    // hal.uartC->printf("target_yaw:%d\n",g.target_yaw);
    motors.set_throttle(channel_throttle->norm_input());

}