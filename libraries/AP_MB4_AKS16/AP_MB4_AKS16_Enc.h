//
// Created by chemmy on 11/12/22.
//

#ifndef ARDUPILOT2_MB4_AKS16_ENC_H
#define ARDUPILOT2_MB4_AKS16_ENC_H


//#include "SPIDevice.h"

#define ENCODER_COUNT   2

#include "AP_HAL/AP_HAL.h"
#include "mb4_1sf_driver.h"

extern const AP_HAL::HAL& hal;


//class MB4_Backend {
//
//};

class AKS16 {
public:
    AKS16() {};
    bool init_AKS16();
    void createBackProcess();
    void update_encoders();
    bool test();

private:
    MB4 mb4;
};


class MB4_AKS16_Enc {
//    friend class MB4_Backend;
public:
    MB4_AKS16_Enc();
    bool init();
    void update();
    int32_t getEnc(int id) { return encoderVal[id]; }

private:

    int32_t encoderVal[ENCODER_COUNT];
    bool  notYetInit;
    AKS16 aks16;
};



#endif //ARDUPILOT2_MB4_AKS16_ENC_H
