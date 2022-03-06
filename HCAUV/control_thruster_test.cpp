#include "HC.h"

bool HC::thruster_test_init()
{
	return true;
}

void HC::thruster_test_run()
{
	// hal.uartC->printf("thruster_test_run\n");
	    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // attitude_control.set_throttle_out(0,true,g.throttle_filt);
        // attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // motors.set_yaw_pwm(g.pwm_yaw_value);
    // motors.set_throttle_pwm(g.pwm_depth_value);
    // motors.set_forward_pwm(g.pwm_forward_value);
    // motors.set_lateral_pwm(g.pwm_lateral_value);
	motors.set_yaw(g.pwm_yaw_value/400);
    motors.set_throttle(g.pwm_depth_value/400);
    motors.set_forward(g.pwm_for_value/400);
    motors.set_lateral(g.pwm_lat_value/400);
	
}

