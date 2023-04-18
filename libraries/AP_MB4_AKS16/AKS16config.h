//
// Created by chemmy on 1/5/23.
//

#ifndef ARDUPILOT2_AKS16CONFIG_H
#define ARDUPILOT2_AKS16CONFIG_H

#define AP_AKS16_AVAILABLE  1

// Params for conversion of encoders to Deg.
#define ERROR_VAL 999.999
#define PERCENT_EXTENDER  5.0
#define MAX_STEP 0.2

// Timeout on getting AKS16 data (in uSec).
#define MAX_LOOP_USEC   400

#define LOG_FREQ    50

#define FREEZE_DURATION_MS  100
#define BAD_ENC_MS      50


#endif //ARDUPILOT2_AKS16CONFIG_H
