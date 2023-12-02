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
    enum status_flags {SCDERR, DELAYERR, AGSERR, SVALID1, SVALID5, MB4_TIMEOUT,
            OTHER_ERROR, BAD_VER_REV, SLAVE_LOC, ENSCD_ERR, SVALID_ERR, BAD_SEMA,
            ENC1_RANGE, ENC2_RANGE,
            ENC1_EX_RANGE, ENC2_EX_RANGE,
            ENC1_STEP, ENC2_STEP,
            ENC1_FREEZE, ENC2_FREEZE,                   // just mark a freeze cycle
            ENC1_FILT_FREEZE, ENC2_FILT_FREEZE,         // Freeze and needs
            ENC1_NOT_VALID, ENC2_NOT_VALID,
            LEVEL_SWASH_PLATE,
            CUSTOM_CTRL};
    AKS16();
    bool init();
    void update();
    bool init_AKS16();
    void createBackProcess();
    void update_encoders(); // backend running process 2nd version
    void recover();
    void Write_MB4();       // write to logger
    void slow_write_mb4(short miligap);   // write to logger with a millis delay
    bool test();            // Test that AKS16 + MB4 are valid
    static const struct AP_Param::GroupInfo var_info[];
//    void printBytes(uint64_t);
    float getEnc1();
    float getEnc2();
    uint32_t getEncStatus();
    static AKS16 *get_singleton() { return _singleton; }

    float getPitch()    { return encDeg1; }
    float getRoll()    { return encDeg2; }

    bool check_mks16_reliable();
    bool isCustCtlBadFlag();
    bool needToLevelSwashPlate();
//    void enableCustomCtrl(bool st);

    void setFlightMode(int v)   { advancedFlightMode = v; }
    uint8_t getFlightMode()     { return advancedFlightMode; }


private:
    bool checkconv_enc_vals(float &e1, float &e2);

    bool  notYetInit;
    bool  seeingMB4;

    static AKS16 *_singleton;

    uint32_t encData1;
    uint32_t encData2;
    float encDeg1;
    float encDeg2;
    float lowPassEnc1;
    float lowPassEnc2;
    MB4 mb4;
    uint32_t encStatus;

    AP_Float    _en1_degMin;
    AP_Int32    _en1_encMin;
    AP_Float    _en1_degMax;
    AP_Int32    _en1_encMax;
    AP_Float    _en2_degMin;
    AP_Int32    _en2_encMin;
    AP_Float    _en2_degMax;
    AP_Int32    _en2_encMax;
    AP_Int8     _enable;

    uint8_t     advancedFlightMode;

    bool        sema;

};

namespace AP {
    AKS16 *aks16();
};


#define SET_BIT(f)  (1 << f)

#endif //ARDUPILOT2_MB4_AKS16_ENC_H
