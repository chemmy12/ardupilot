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
    void update_encoders();

private:
    // const int dataReadyPin = 6;
    uint32_t currentTime, displayTime;
    const int chipSelectPin = 53;
    uint8_t StatusInformationF0;
    uint8_t StatusInformationF1;
    uint8_t StatusInformationF5;
    uint8_t StatusInformationF0_1;
    uint8_t StatusInformationF0_2;
    uint8_t StatusInformationF0_3;
    uint32_t SCDATA1;
    uint32_t SCDATA5;
    uint32_t OUT1;
    uint32_t OUT2;
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
//    bool _add_backend(MB4_Backend *backend);
//    MB4_Backend *probe(MB4_AKS16_Enc, AP_HAL::OwnPtr<AP_HAL::SPIDevice>);

    int32_t encoderVal[ENCODER_COUNT];
    uint32_t counter;
//    AP_HAL::OwnPtr<AP_HAL::SPIDevice> spiDevp;
    bool  notYetInit;
    AKS16 aks16;
};



#endif //ARDUPILOT2_MB4_AKS16_ENC_H
