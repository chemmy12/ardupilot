#include "AC_CustomControl_PID.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

#if CUSTOMCONTROL_PID_ENABLED

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_PID::var_info[] = {
    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_pid_angle_roll2, "ANG_RLL_", 1, AC_CustomControl_PID, AC_PID),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @Range{Sub}: 0.0 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_pid_angle_pitch2, "ANG_PIT_", 2, AC_CustomControl_PID, AC_PID),

    // @Param: ROLL_D2C
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("ROLL_D2C", 3, AC_CustomControl_PID, roll_kd2c, 0),

    // @Param: PITCH_D2C
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("PITCH_D2C", 4, AC_CustomControl_PID, pitch_kd2c, 0),

	// @Param: D2C_MAX
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D2C_MAX", 5, AC_CustomControl_PID, d2c_max, 0),

    AP_GROUPEND
};

AC_CustomControl_PID::AC_CustomControl_PID(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Heli*& att_control, AP_MotorsHeli*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _pid_angle_roll2(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, 0.0f, AC_ATC_HELI_RATE_RP_IMAX, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_HELI_RATE_RP_FILT_HZ, dt),
    _pid_angle_pitch2(AC_ATC_HELI_RATE_RP_P, AC_ATC_HELI_RATE_RP_I, AC_ATC_HELI_RATE_RP_D, 0.0f, AC_ATC_HELI_RATE_RP_IMAX, AC_ATC_HELI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_HELI_RATE_RP_FILT_HZ, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

Vector3f AC_CustomControl_PID::update()
{
      // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // run custom controller after here
     Quaternion attitude_body, attitude_target;
    float pitch = aks16->getPitch();
    float roll = aks16->getRoll();


//    _ahrs->get_quat_body_to_ned(attitude_body);

    _ahrs->get_quat_body_to_ned(attitude_body);

    attitude_target = _att_control->get_attitude_target_quat();


    Vector3f attitude_euler;

    attitude_target.to_euler(attitude_euler[0],attitude_euler[1],attitude_euler[2]);

//    gcs().send_text(MAV_SEVERITY_INFO, "target_pitch=%f", radians(_p_angle_roll2.kP()));

    // run rate controller
//    Vector3f encoder_latest = _ahrs->get_gyro_latest();
    Vector3f motor_out;
    //_pid_angle_roll2.set_dt(0.0025);
    //_pid_angle_pitch2.set_dt(0.0025);
    motor_out.x = _pid_angle_roll2.update_all(degrees(attitude_euler[0]), roll, false);
    motor_out.y = _pid_angle_pitch2.update_all(degrees(attitude_euler[1]), pitch, false);
	// Cross of gyroscopic effect
	float roll_d2c = constrain_float(_pid_angle_pitch2.get_d() * roll_kd2c , -d2c_max , d2c_max);
    float pitch_d2c = constrain_float(_pid_angle_roll2.get_d() * pitch_kd2c , -d2c_max , d2c_max);
    motor_out.x += roll_d2c;
    motor_out.y += pitch_d2c;
    motor_out.z = 0;

    static unsigned int divideLogSpeed = 0;

    if (divideLogSpeed++ % 20)
        return motor_out;

//    hal.console->printf("Pitch enc=%f, euler=%f, euler deg=%f motor.c=%f\n", pitch, attitude_euler[1], degrees(attitude_euler[1]), motor_out.y);
    AP::logger().Write("CCLR", "TimeUS,cmd,sts,p,i,d,d2c,ff,err",
                        "S--------", "F--------", "Qffffffff", // units, multi, format
                        AP_HAL::micros64(),
                        degrees(attitude_euler[0]),
                        roll,
                        _pid_angle_roll2.get_p(),
                        _pid_angle_roll2.get_i(),
                        _pid_angle_roll2.get_d(),
						roll_d2c,
                        _pid_angle_roll2.get_ff(),
                        _pid_angle_roll2.get_error()
                        );
    AP::logger().Write("CCLP", "TimeUS,cmd,sts,p,i,d,d2c,ff,err",
                        "S--------", "F--------", "Qffffffff", // units, multi, format
                        AP_HAL::micros64(),
                        degrees(attitude_euler[1]),
                        pitch,
                        _pid_angle_pitch2.get_p(),
                        _pid_angle_pitch2.get_i(),
                        _pid_angle_pitch2.get_d(),
						pitch_d2c,
                        _pid_angle_pitch2.get_ff(),
                        _pid_angle_pitch2.get_error()
                        );

//    hal.console->printf("AC_Custom_pid: pitch=%f, roll=%f, x=%f, y=%f", pitch, roll, motor_out.x, motor_out.y);

    return motor_out;
}

// This example uses exact same controller architecture as ArduCopter attitude controller without all the safe guard against saturation.
// The gains are scaled 0.9 times to better detect switch over response. 
// Note that integrator are not reset correctly as it is done in reset_main_att_controller inside AC_CustomControl.cpp
// This is done intentionally to demonstrate switch over performance of two exact controller with different reset handling.
void AC_CustomControl_PID::reset(void)
{
    _pid_angle_roll2.reset_I();
    _pid_angle_pitch2.reset_I();
    _pid_angle_roll2.reset_filter();
    _pid_angle_pitch2.reset_filter();
}

#endif
