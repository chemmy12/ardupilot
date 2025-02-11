#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AC_PID/AC_P.h>

#include "AC_CustomControl_Backend.h"

#include "AP_MB4_AKS16/AKS16.h"

#ifndef CUSTOMCONTROL_PID_ENABLED
    #define CUSTOMCONTROL_PID_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_PID_ENABLED

class AC_CustomControl_PID : public AC_CustomControl_Backend {
public:
    AC_CustomControl_PID(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Heli*& att_control, AP_MotorsHeli*& motors, float dt);

    // run lowest level body-frame rate controller and send outputs to the motors
    Vector3f update() override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // put controller related variable here

    // angle P controller  objects
    AC_PID                _pid_angle_roll2;
    AC_PID                _pid_angle_pitch2;

	AP_Float roll_kd2c;
	AP_Float pitch_kd2c;
	AP_Float d2c_max;

    //AC_PID                _p_angle_yaw2;

	// rate PID controller  objects
    //AC_PID _pid_atti_rate_roll;
    //AC_PID _pid_atti_rate_pitch;
    //AC_PID _pid_atti_rate_yaw;

    AKS16 *aks16 = AKS16::get_singleton();
};

#endif
