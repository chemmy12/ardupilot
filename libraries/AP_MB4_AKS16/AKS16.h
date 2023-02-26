//
// Created by chemmy on 11/12/22.
//

#ifndef ARDUPILOT2_MB4_AKS16_ENC_H
#define ARDUPILOT2_MB4_AKS16_ENC_H


//#include "SPIDevice.h"

#define ENCODER_COUNT   2

#include "AKS16config.h"

#include "AP_HAL/AP_HAL.h"
#include <AP_Param/AP_Param.h>
#include "mb4_1sf_driver.h"

extern const AP_HAL::HAL& hal;


class AKS16 {
public:
    enum status_flags {SCDERR, DELAYERR, AGSERR, SVALID1, SVALID5, ENC1RANGE, ENC2RANGE, ENC1STEP, ENC2STEP, MB4_TIMEOUT, OTHER_ERROR, BAD_VER_REV, SLAVE_LOC, ENSCD_ERR, SVALID_ERR, BAD_SEMA};
    AKS16();
    bool init();
    void update();
    bool init_AKS16();
    void createBackProcess();
    void update_encoders(); // backend running process 2nd version
    void recover();
    void Write_MB4();       // write to logger
    bool test();            // Test that AKS16 + MB4 are valid
    static const struct AP_Param::GroupInfo var_info[];
    void printBytes(uint64_t);
    float getEnc1();
    float getEnc2();
    int16_t getEncStatus();
    static AKS16 *get_singleton() { return _singleton; }


private:
    bool checkconv_enc_vals(float &e1, float &e2);

    bool  notYetInit;
    bool  seeingMB4;

    static AKS16 *_singleton;

    uint8_t StatusInformationF1;
    uint8_t StatusInformationF5;
    uint8_t StatusInformationF0_1;
    uint8_t StatusInformationF0_2;
    uint8_t StatusInformationF0_3;
    uint32_t encData1;
    uint32_t encData2;
    float encDeg1;
    float encDeg2;
    MB4 mb4;
    uint16_t encStatus;

    AP_Float    _en1_degMin;
    AP_Int32    _en1_encMin;
    AP_Float    _en1_degMax;
    AP_Int32    _en1_encMax;
    AP_Float    _en2_degMin;
    AP_Int32    _en2_encMin;
    AP_Float    _en2_degMax;
    AP_Int32    _en2_encMax;
    AP_Int8     _enable;

    bool        sema;

};

namespace AP {
    AKS16 *aks16();
};


#define SET_BIT(f)  (1 << f)

#endif //ARDUPILOT2_MB4_AKS16_ENC_H
