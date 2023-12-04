//
// Created by chemmy on 1/5/23.
//

#ifndef ARDUPILOT2_AKS16CONFIG_H
#define ARDUPILOT2_AKS16CONFIG_H

#define AP_AKS16_AVAILABLE  1

#define AKS_DRIVER_FREQ     800.0

// Params for conversion of encoders to Deg.
//#define ERROR_VAL 999.999
#define PERCENT_EXTENDER    10.0
#define MAX_STEP            0.75               // A.K.A. Spike
#define MAX_EX_RANGE_MS     20

// Timeout on getting AKS16 data (in uSec).
#define MAX_LOOP_USEC       400

#define LOG_FREQ            100

#define FREEZE_DURATION_MS  20
#define MAX_FREEZE_MS       20
#define BAD_ENC_MS          50

// low pass filter params
//#define LOW_PASS_FREQ   10



#endif //ARDUPILOT2_AKS16CONFIG_H
