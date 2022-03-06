#include "HC.h"

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float throttle_in;
    float angle_boost;
    float throttle_out;
    float throttle_hover;
    float desired_alt;
    float inav_alt;
    float baro_alt;
    int16_t desired_rangefinder_alt;
    int16_t rangefinder_alt;
    float terr_alt;
    int16_t target_climb_rate;
    int16_t climb_rate;
};

// struct PACKED log_Control_Tuning {
//     LOG_PACKET_HEADER;
//     uint64_t time_us;    //数据项
//     int16_t HC_YAW_ANG_TARGET;
//     float HC_YAW_ANG_Error;
//     float HC_YAW_ANG_CURRENT;
//     float HC_YAW_RATE_TARGET;
//     float HC_YAW_RATE_CURRENT;
//     float HC_YAW_RATE_ERROR;
//     float HC_YAW_force;
// };

// Write a control tuning packet
void HC::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    terrain.height_above_terrain(terr_alt, true);
#endif

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us : AP_HAL::micros64(),
        // throttle_in         : attitude_control.get_throttle_in(),
        throttle_in : (float)g.target_yaw,
        angle_boost : attitude_control.angle_boost(),
        throttle_out : motors.get_throttle(),
        throttle_hover : motors.get_throttle_hover(),
        desired_alt : pos_control.get_alt_target() / 100.0f,
        inav_alt : inertial_nav.get_altitude() / 100.0f,
        baro_alt : barometer.get_altitude(),
        desired_rangefinder_alt : (int16_t)target_rangefinder_alt,
        rangefinder_alt : rangefinder_state.alt_cm,
        terr_alt : terr_alt,
        target_climb_rate : (int16_t)pos_control.get_vel_target_z(),
        climb_rate : climb_rate
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write a control tuning packet
// void HC::Log_Write_Control_Tuning()
// {
// //     // get terrain altitude
// //     float terr_alt = 0.0f;
// // #if AP_TERRAIN_AVAILABLE && AC_TERRAIN
// //     terrain.height_above_terrain(terr_alt, true);
// // #endif

//     struct log_Control_Tuning pkt = {
//         LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
// 		time_us : AP_HAL::micros64(),
// 		HC_YAW_ANG_TARGET : g.target_yaw,
// 		HC_YAW_ANG_Error : attitude_control.get_hc_yaw_ang_error(),
//         HC_YAW_ANG_CURRENT : ahrs.get_yaw(),
//         HC_YAW_RATE_TARGET : attitude_control.rate_bf_targets().z,
//         HC_YAW_RATE_CURRENT : ahrs.get_gyro().z,
//         HC_YAW_RATE_ERROR :   attitude_control.get_vel_error(),
//         HC_YAW_force :  motors.get_yaw_force()
//     };
//     logger.WriteBlock(&pkt, sizeof(pkt));
// }

// Write an attitude packet
void HC::Log_Write_Attitude()
{
    Vector3f targets = attitude_control.get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    logger.Write_Attitude(ahrs, targets);

    AP::ahrs_navekf().Log_Write();
    logger.Write_AHRS2(ahrs);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    logger.Write_POS(ahrs);
}

//log_Test
struct PACKED log_HC
{
    /* data */
    LOG_PACKET_HEADER; //日志包头
    uint64_t time_us;  //数据项
    float HC_robust_yaw_force;
    float HC_robust_yaw_error;
};
void HC::Log_write_HC()
{
    struct log_HC pkt_hc = {
        LOG_PACKET_HEADER_INIT(LOG_HC_MSG),
        time_us : AP_HAL::micros64(),
        HC_robust_yaw_force : hc_yaw_robust_force,
        HC_robust_yaw_error : hc_yaw_robust_error
    };
    logger.WriteBlock(&pkt_hc, sizeof(pkt_hc));
}
struct PACKED log_DVL
{
    /* data */
    LOG_PACKET_HEADER; //日志包头
    uint64_t time_us;  //数据项
    float VX;
    float VY;
    float VZ;
    float DISTANCE;
    float ROLL;
    float PITCH;
    float YAW;
};

void HC::Log_write_DVL()
{
    struct log_DVL pkt_dvl = {
        LOG_PACKET_HEADER_INIT(LOG_DVL_MSG),
        time_us : AP_HAL::micros64(),
        VX : hc_dvl50_vel_x,
        VY : hc_dvl50_vel_y,
        VZ : hc_dvl50_vel_z,
        DISTANCE : hc_dvl50_distacne,
        ROLL : hc_dvl50_roll,
        PITCH :   hc_dvl50_pitch,
        YAW : hc_dvl50_yaw
    };
    logger.WriteBlock(&pkt_dvl, sizeof(pkt_dvl));
    // hal.uartC->printf("g.target_yaw:%d,attitude_control:%f,ahrs.get_yaw():%f\n",g.target_yaw,attitude_control.get_hc_yaw_ang_error(),ahrs.get_yaw());
}


struct PACKED log_HC_PID
{
    /* data */
    LOG_PACKET_HEADER; //日志包头
    uint64_t time_us;  //数据项
    float HC_YAW_ANG_TARGET;
    float HC_YAW_ANG_Error;
    float HC_YAW_ANG_CURRENT;
    // float HC_YAW_RATE_TARGET;
    // float HC_YAW_RATE_CURRENT;
    // float HC_YAW_RATE_ERROR;
    float HC_YAW_force;
};
void HC::Log_write_HC_PID()
{
    struct log_HC_PID pkt_hc_pid = {
        LOG_PACKET_HEADER_INIT(LOG_HC_PID_MSG),
        time_us : AP_HAL::micros64(),
        HC_YAW_ANG_TARGET : (float)g.target_yaw,
        HC_YAW_ANG_Error : attitude_control.get_hc_yaw_ang_error(),
        HC_YAW_ANG_CURRENT : ahrs.get_yaw(),
        // HC_YAW_RATE_TARGET : attitude_control.rate_bf_targets().z,
        // HC_YAW_RATE_CURRENT : ahrs.get_gyro().z,
        // HC_YAW_RATE_ERROR :   attitude_control.get_vel_error(),
        HC_YAW_force : motors.get_yaw_force()
    };
    logger.WriteBlock(&pkt_hc_pid, sizeof(pkt_hc_pid));
    // hal.uartC->printf("g.target_yaw:%d,attitude_control:%f,ahrs.get_yaw():%f\n",g.target_yaw,attitude_control.get_hc_yaw_ang_error(),ahrs.get_yaw());
}

struct PACKED log_HC_Robust
{
    /* data */
    LOG_PACKET_HEADER; //日志包头
    uint64_t time_us;  //数据项
    float HC_YAW_ANG_TARGET_ROBUST;
    float HC_YAW_ANG_Error_ROBUST;
    float HC_YAW_ANG_CURRENT_ROBUST;
    float HC_YAW_force_ROBUST;
};
void HC::Log_write_HC_Robust()
{
    struct log_HC_Robust pkt_hc_robust = {
        LOG_PACKET_HEADER_INIT(LOG_HC_ROBUST_MSG),
        time_us : AP_HAL::micros64(),
        HC_YAW_ANG_TARGET_ROBUST : (float)g.target_yaw,
        HC_YAW_ANG_Error_ROBUST : hc_yaw_robust_error,
        HC_YAW_ANG_CURRENT_ROBUST : ahrs.get_yaw(),
        HC_YAW_force_ROBUST : motors.get_yaw_force()
    };
    logger.WriteBlock(&pkt_hc_robust, sizeof(pkt_hc_robust));
    // hal.uartC->printf("g.target_yaw:%d,attitude_control:%f,ahrs.get_yaw():%f\n",g.target_yaw,attitude_control.get_hc_yaw_ang_error(),ahrs.get_yaw());
}

struct PACKED log_MotBatt
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float lift_max;
    float bat_volt;
    float bat_res;
    float th_limit;
};

// Write an rate packet
void HC::Log_Write_MotBatt()
{
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us : AP_HAL::micros64(),
        lift_max : (float)(motors.get_lift_max()),
        bat_volt : (float)(motors.get_batt_voltage_filt()),
        bat_res : (float)(battery.get_resistance()),
        th_limit : (float)(motors.get_throttle_limit())
    };
    logger.WriteBlock(&pkt_mot, sizeof(pkt_mot));
}

// Wrote an event packet
void HC::Log_Write_Event(Log_Event id)
{
    logger.Write_Event(id);
}

struct PACKED log_Data_Int16t
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void HC::Log_Write_Data(uint8_t id, int16_t value)
{
    if (should_log(MASK_LOG_ANY))
    {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us : AP_HAL::micros64(),
            id : id,
            data_value : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION
void HC::Log_Write_Data(uint8_t id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY))
    {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us : AP_HAL::micros64(),
            id : id,
            data_value : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void HC::Log_Write_Data(uint8_t id, int32_t value)
{
    if (should_log(MASK_LOG_ANY))
    {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us : AP_HAL::micros64(),
            id : id,
            data_value : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void HC::Log_Write_Data(uint8_t id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY))
    {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us : AP_HAL::micros64(),
            id : id,
            data_value : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void HC::Log_Write_Data(uint8_t id, float value)
{
    if (should_log(MASK_LOG_ANY))
    {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us : AP_HAL::micros64(),
            id : id,
            data_value : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

// logs when baro or compass becomes unhealthy
void HC::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy())
    {
        sensor_health.baro = barometer.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::BARO, (sensor_health.baro ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy())
    {
        sensor_health.compass = compass.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::COMPASS, (sensor_health.compass ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }
}

struct PACKED log_GuidedTarget
{
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
void HC::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f &pos_target, const Vector3f &vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us : AP_HAL::micros64(),
        type : target_type,
        pos_target_x : pos_target.x,
        pos_target_y : pos_target.y,
        pos_target_z : pos_target.z,
        vel_target_x : vel_target.x,
        vel_target_y : vel_target.y,
        vel_target_z : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure HC::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    {LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
     // "HC_PID",  "Qhffffff",   "TimeUS,ATar,AErr,ACurr,RateTar,RateCurr,RateErr,YawF",	"s-------", "F0000000"  },
     "CTUN", "Qfffffffccfhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00BBBBBB"},
    {LOG_MOTBATT_MSG, sizeof(log_MotBatt),
     "MOTB", "Qffff", "TimeUS,LiftMax,BatVolt,BatRes,ThLimit", "s-vw-", "F-00-"},
    {LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),
     "D16", "QBh", "TimeUS,Id,Value", "s--", "F--"},
    {LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),
     "DU16", "QBH", "TimeUS,Id,Value", "s--", "F--"},
    {LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),
     "D32", "QBi", "TimeUS,Id,Value", "s--", "F--"},
    {LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),
     "DU32", "QBI", "TimeUS,Id,Value", "s--", "F--"},
    {LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),
     "DFLT", "QBf", "TimeUS,Id,Value", "s--", "F--"},
    {LOG_HC_MSG, sizeof(log_HC),
     "HC_ANGLE", "Qff", "TimeUS,Value,Value", "s--", "F--"},
    {LOG_HC_PID_MSG, sizeof(log_HC_PID),
    "HC_PID", "Qffff", "TimeUS,ATar,AErr,ACurr,YawF", "s----", "F0000"},
    //  "HC_PID", "Qfffffff", "TimeUS,ATar,AErr,ACurr,RateTar,RateCurr,RateErr,YawF", "s-------", "F0000000"},
    {LOG_HC_ROBUST_MSG, sizeof(log_HC_Robust),
     "HC_ROBUST", "Qffff", "TimeUS,ATar,AErr,ACurr,YawF", "s----", "F0000"},
    {LOG_DVL_MSG, sizeof(log_DVL),
     "DVL", "Qfffffff", "TimeUS,Vx,Vy,Vz,Dis,Roll,Pitch,Yaw", "s-------", "F0000000"},     
    {LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
     "GUID", "QBffffff", "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-000000"},
};

void HC::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    logger.Write_Mode(control_mode, control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void HC::log_init()
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void HC::Log_Write_Control_Tuning() {}
void HC::Log_Write_Performance() {}
void HC::Log_Write_Attitude(void) {}
void HC::Log_Write_MotBatt() {}
void HC::Log_Write_Event(Log_Event id) {}
void HC::Log_Write_Data(uint8_t id, int32_t value) {}
void HC::Log_Write_Data(uint8_t id, uint32_t value) {}
void HC::Log_Write_Data(uint8_t id, int16_t value) {}
void HC::Log_Write_Data(uint8_t id, uint16_t value) {}
void HC::Log_Write_Data(uint8_t id, float value) {}
void HC::Log_Sensor_Health() {}
void HC::Log_Write_GuidedTarget(uint8_t target_type, const Vector3f &pos_target, const Vector3f &vel_target) {}
void HC::Log_Write_Vehicle_Startup_Messages() {}

void HC::log_init(void) {}

#endif // LOGGING_ENABLED
