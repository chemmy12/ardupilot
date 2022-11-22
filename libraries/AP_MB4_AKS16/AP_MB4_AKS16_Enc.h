//
// Created by chemmy on 11/12/22.
//

#ifndef ARDUPILOT2_MB4_AKS16_ENC_H
#define ARDUPILOT2_MB4_AKS16_ENC_H

#include "mb4_1sf_driver.h"
#include "AP_HAL/AP_HAL.h"
//#include "SPIDevice.h"

#define ENCODER_COUNT   2

extern const AP_HAL::HAL& hal;

class MB4_Backend {

};

class MB4_AKS16_Enc: public MB4_Backend {
    friend class MB4_Backend;
public:
    MB4_AKS16_Enc();
    bool init();
    void update();
    int32_t getEnc(int id) { return encoderVal[id]; }

private:
//    bool _add_backend(MB4_Backend *backend);
//    MB4_Backend *probe(MB4_AKS16_Enc, AP_HAL::OwnPtr<AP_HAL::SPIDevice>);

    int32_t encoderVal[ENCODER_COUNT];
    uint32_t counter;
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> spiDevp;
    bool  notYetInit;
};



#endif //ARDUPILOT2_MB4_AKS16_ENC_H
