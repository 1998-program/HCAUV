/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ArduSub scheduling, originally copied from ArduCopter

#include "Sub.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(fifty_hz_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &sub.optflow,             update,         200, 160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(update_turn_counter,   10,     50),
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90),
    SCHED_TASK_CLASS(AP_Notify,           &sub.notify,       update,              50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550),
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update_trigger,      50,  75),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(AP_Logger,     &sub.logger,    periodic_tasks,     400, 300),
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50),
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
    SCHED_TASK_CLASS(Compass,          &sub.compass,              cal_update, 100, 100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &sub.g2.gripper,       update,              10,  75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
};

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::setup()
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();
//    hal.uartD->printf("Sub::setup");

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
	init_mod_ciscrea();
	hal.uartD->printf("ciscrea_A:%f\n",CIS_A[3]);
	hal.uartD->printf("X1_N:%f\n",X1_N);
}

void Sub::loop()
{
	//loop运行起来 如何调度？
    scheduler.loop();
	//400HZ 2.5ms
    G_Dt = scheduler.get_loop_period_s();

}


// Main loop - 400hz
void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    //don't run rate controller in manual or motordetection modes
    if (control_mode != MANUAL && control_mode != MOTOR_DETECT) {
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // send outputs to the motors library
    motors_output();
	
	cal_ciscrea_angle();
//	hal.uartC->printf("real_angle:%f\n",real_angle);
//	hal.uartC->printf("real_angle:%f\n",torque_test);

	send_to_rasp();
//	hal.uartC->printf("send_to_rasp\n");

	receive_from_rasp();
//
//	hal.uartC->printf("receive_from_rasp\n");


	//uart test
//	uart_test();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// 50 Hz tasks
void Sub::fifty_hz_loop()
{
    // check pilot input failsafe
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();

    // Update rc input/output
    rc().read_input();
    SRV_Channels::output_ch_all();
	
//	if (should_log(MASK_LOG_HC)) {
//        Log_write_HC();
//    }
	
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass()
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().enabled()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        logger.Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && mode_requires_GPS(control_mode)) {
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        logger.Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        logger.Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        logger.Write_IMU();
    }
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

    ServoRelayEvents.update_events();
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.update_orientation();

        // set all throttle channel settings
        motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // update position controller alt limits
    update_poscon_alt_max();

    // log terrain data
    terrain_logging();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());
}

// called at 50hz
void Sub::update_GPS()
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            gps_updated = true;
            break;
        }
    }

    if (gps_updated) {
#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Sub::read_AHRS()
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update(true);
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

bool Sub::control_check_barometer()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
#endif
    return true;
}
//CISCREA para init
void Sub::init_mod_ciscrea(){
//	X1_N = ahrs.yaw_sensor;
	X1_N = 0.0;
	X2_N = 0.0;
	X1_N_1 = 0.0;
	X2_N_1 = 0.0;
	torque = 0.0;
	target_angle = 5.0;
	f_h1_flag = 0;                          // 接收到帧头的第一个字节标志位
    f_h_flag = 0;                           // 接收到帧头标志位
    f_t1_flag = 0;                          // 接收到帧尾的第一个字节标志位
    real_angle = 0.0;
    
}
void Sub::cal_ciscrea_angle(){
	X1_N_1 = CIS_A[0] * X1_N + CIS_A[1] * X2_N + 0.0;
	X2_N_1 = CIS_A[2] * X1_N + CIS_A[3] * X2_N + CIS_B[1] * torque;
//	hal.uartC->printf("X1_N:%f\n",X1_N);
	real_angle = CIS_C[0] * X1_N;
	X1_N = X1_N_1;
	X2_N = X2_N_1;
}


void Sub::send_to_rasp(){
//	float code_torque = 0.0;
//	real_angle = 456.12378;
//	code_torque = real_angle;
	float error_angle = target_angle - real_angle;
	
	tran_angle.angleX = error_angle;
	hal.uartC->printf("real_angle:%f\n",real_angle);
	
//	_buffertx[0] = 0x3A;
//	_bufferrx[1] = 0x3B;
//	_bufferrx[2] = 0x11;
//	_bufferrx[3] = 0x12;
//	_bufferrx[4] = 0x12;
//	_bufferrx[5] = 0x20;
//	_bufferrx[6] = 0x7E;
//	_bufferrx[7] = 0x7F;


	_buffertx[0] = 0x3A;
	_bufferrx[1] = 0x3B;
	_bufferrx[2] = tran_angle.angle_char[0];
	_bufferrx[3] = tran_angle.angle_char[1];
	_bufferrx[4] = tran_angle.angle_char[2];
	_bufferrx[5] = tran_angle.angle_char[3];
	_bufferrx[6] = 0x7E;
	_bufferrx[7] = 0x7F;
	hal.uartC->printf("error_angle:%f\n",error_angle);
	
//	int i = 0;
//	while(i < 8){
//		hal.uartD->write(_buffertx[i]);
//		hal.scheduler->delay(10);
//		i += 1;
//	}		
	hal.uartD->write(_buffertx[0]);
	hal.uartD->write(_bufferrx[1]);
	hal.uartD->write(_bufferrx[2]);
	hal.uartD->write(_bufferrx[3]);
	hal.uartD->write(_bufferrx[4]);
	hal.uartD->write(_bufferrx[5]);
	hal.uartD->write(_bufferrx[6]);
	hal.uartD->write(_bufferrx[7]);
//	for(int j = 0; j < 8; j++){
//		hal.uartD->printf("%c\n",_buffertx[i]);
//		hal.scheduler->delay(10);
//	}
//	hal.uartD->write(const uint8_t * buffer, size_t size)

	_buffertx[0] = 0x00;
	_bufferrx[1] = 0x00;
	_bufferrx[2] = 0x00;
	_bufferrx[3] = 0x00;
	_bufferrx[4] = 0x00;
	_bufferrx[5] = 0x00;
	_bufferrx[6] = 0x00;
	_bufferrx[7] = 0x00;

//	hal.uartC->printf("real_angle:%f\n",real_angle);


}
void Sub::receive_from_rasp(){
	int16_t numc;

	numc = hal.uartD->available();
	
//	hal.uartD->printf("numc:%d\n",numc);




	int tnum = 0;
	if(numc){
		while (tnum < numc){
			_bufferrx[tnum] = hal.uartD->read();
//			hal.uartD->printf("receive:%c\n",_buffer[i]);
			if(f_h_flag == 1){     //有帧头，判断帧尾，接收消息
				if (f_t1_flag == 1){   //有帧头，有帧尾1
					if(_bufferrx[tnum] == Frame_Tail2){
						int i = 0;
						for(i = 2;i < (tnum -1);i++){
							tran_force.force_char[i-2] = _bufferrx[i];
						}
						torque = tran_force.forceX;
//                        for (i = 2; i < (tnum - 1); i++)
//                        {
//                            hal.uartC->write(_bufferrx[i]);	// 通过串口发送字节
//                        }
						
						tnum = 0;						

					}
					else{
						f_t1_flag = 0;
						tnum++;						
					}
				}
				else{    // 有帧头，无帧尾1
					if(_bufferrx[tnum] == Frame_Tail1){
						f_t1_flag = 1;
						tnum++;
					}
					else{
						tnum++;
					}
				}
					
			}
			else{
				if(f_h1_flag == 1){
					if(_bufferrx[tnum] == Frame_Header2){
						f_h_flag = 1;
						tnum++;
					}
					else{
						f_h1_flag = 0;
						tnum = 0;
					}
				}
				else{
					if(_bufferrx[tnum] == Frame_Header1){
						f_h1_flag = 1;
						tnum++;
					}
					else{
						tnum = 0;
					}
				}
			}
			
		}
	}
//	if(hc_decode(numc)){}	
//	hal.uartD->printf("receive_real_angle:%f\n",real																																																																																																																																																						_angle);

	//处理程序
	
//	hc_decode(numc);
//	if(_bufferrx[0] == Frame_Header1 && _bufferrx[1] == Frame_Header2 && _bufferrx[6] == Frame_Tail1 && _bufferrx[7] == Frame_Tail2){
//
//
//		hal.uartD->write(_bufferrx[0]);
//		hal.uartD->write(_bufferrx[1]);																																																																																																																																																																																						
//
//
//		hal.uartD->write(_bufferrx[2]);
//		hal.uartD->write(_bufferrx[3]);																																																																																																																																																																																						
//		hal.uartD->write(_bufferrx[4]);
//		hal.uartD->write(_bufferrx[5]);
//
//		hal.uartD->write(_bufferrx[6]);
//		hal.uartD->write(_bufferrx[7]);
//	}
	

	
//	while(i < numc){
//		_bufferrx[i] = 0;
//	}
	_bufferrx[0] = 0x00;
	_bufferrx[1] = 0x00;
	_bufferrx[2] = 0x00;
	_bufferrx[3] = 0x00;
	_bufferrx[4] = 0x00;
	_bufferrx[5] = 0x00;
	_bufferrx[6] = 0x00;
	_bufferrx[7] = 0x00;

//	hal.uartD->printf("_bufferrx is 0 ?:%d",int(_bufferrx[3]));
	

}

void Sub::uart_test(){
//	hal.uartD->printf("123");
}

void Sub::hc_code(){
//	float code_torque = 0.0;
//	_buffertx[0] = '$'; //head
//	_buffertx[1] = '4'; //LEN
//	_buffertx[2] = '0'; //ID
//	_bufferrx[3] = tran_angle.[0];
//	_bufferrx[4] = tran1.data[1];
//	_bufferrx[5] = tran1.data[2];
//	_bufferrx[6] = tran1.data[3];
//	_bufferrx[7] = '*';
}

//void Sub::hc_decode(int16_t numc){
//	if(numc > 0){
//		_bufferrx[tnum] == 
//	}	
//
//}

//bool Sub::hc_decode(int16_t numc){
//	uint8_t len = 0;
//	uint8_t ID;
//	bool flag = false;
//	float number = 0.0;
//	
//	if(_bufferrx[0] == '$'){
//		uint16_t j = 10;
//		len = int(_bufferrx[1]) - 48;
//		ID = _bufferrx[2];
//		for(int i = 1; i <= len; i++){
//			if(_bufferrx[2 + i] == '.'){
//				flag = 1;
//			}
//			else{
//				if(flag == 0){
//					number = number *10 + (int(_bufferrx[2 + i])-48);
//				}
//				else{
//					number = number + (float(_bufferrx[2 + i])-48) / j;
//					j = j * 10;
//				}
//			}
////			hal.uartD->printf("_bufferrx:%d",(int(_bufferrx[2 + i])-48));
//		}
//		switch (ID){
//			case '0':
//					real_angle = number;break;
//
//			case '1':
//					torque = number;break;
//			}
//		flag = true;
//		
//		return true;
//		
//	} 
//	else return false;
//	
//	return false;
//}



AP_HAL_MAIN_CALLBACKS(&sub);
