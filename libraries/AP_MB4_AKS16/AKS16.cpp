// the sensor communicates using SPI, so include the library:
#include "AKS16.h"
#include "mb4_1sf_driver.h"

#define SEMA_AVAIL true
#define SEMA_NOT_AVAIL false
const AP_Param::GroupInfo AKS16::var_info[] = {

#ifdef NOTTOCOMPILE
        // @Param: ENABLE
        // @DisplayName: AKS calib
        // @Description: AKS calib
        // @Values: 0:Disabled, 1:Enabled
        // @User: Standard
        AP_GROUPINFO_FLAGS("ENABLE", 10, AKS16, _enable, 0, AP_PARAM_FLAG_ENABLE),
#endif

        // @Param: en1_degMin
        // @DisplayName: encoder1 degrees at left side
        // @Description: measured angle for encoder1 at extreme left
        // @Range: -90 90
        // @Units: degrees
        // @Increment: 0.001
        // @User: Advanced
        AP_GROUPINFO("en1_degMin",    2, AKS16, _en1_degMin, 0),

        // @Param: en1_encMin
        // @DisplayName: encoder1 readings at left side
        // @Description: the actual encoder1 reading when placed at extreme left
        // @Range: 0 16000000
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("en1_encMin",    3, AKS16, _en1_encMin, 1111111),

        // @Param: en1_degMax
        // @DisplayName: encoder1 degrees at right side
        // @Description: measured angle for encoder1 at extreme right
        // @Range: -90 90
        // @Units: degrees
        // @Increment: 0.001
        // @User: Advanced
        AP_GROUPINFO("en1_degMax",    4, AKS16, _en1_degMax, 0),

        // @Param: en1_encMax
        // @DisplayName: encoder1 readings at right side
        // @Description: the actual encoder1 reading when placed at extreme right
        // @Range: 0 16000000
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("en1_encMax",    5, AKS16, _en1_encMax, 1111111),

        // @Param: en2_degMin
        // @DisplayName: encoder1 degrees at left side
        // @Description: measured angle for encoder1 at extreme left
        // @Range: -90 90
        // @Units: degrees
        // @Increment: 0.001
        // @User: Advanced
        AP_GROUPINFO("en2_degMin",    6, AKS16, _en2_degMin, 0),

        // @Param: en2_encMin
        // @DisplayName: encoder1 readings at left side
        // @Description: the actual encoder1 reading when placed at extreme left
        // @Range: 0 16000000
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("en2_encMin",    7, AKS16, _en2_encMin, 1111111),

        // @Param: en2_degMax
        // @DisplayName: encoder1 degrees at right side
        // @Description: measured angle for encoder1 at extreme right
        // @Range: -90 90
        // @Units: degrees
        // @Increment: 0.001
        // @User: Advanced
        AP_GROUPINFO("en2_degMax",    8, AKS16, _en2_degMax, 0),

        // @Param: en2_encMax
        // @DisplayName: encoder1 readings at right side
        // @Description: the actual encoder1 reading when placed at extreme right
        // @Range: 0 16000000
        // @Increment: 1
        // @User: Advanced
        AP_GROUPINFO("en2_encMax",    9, AKS16, _en2_encMax, 1111111),

        AP_GROUPEND
};


AKS16::AKS16():
        notYetInit(true), sema(SEMA_NOT_AVAIL)
{
    _enable.set(0);
}


void AKS16::update()
{

    if (notYetInit) {    // happens only once at boot
        init_AKS16();
        createBackProcess();
        notYetInit = false;
        sema = SEMA_AVAIL;
    }

//    hal.console->printf("AKS16::update(): Micros=%ld. m64=%lld\n", AP_HAL::micros(),  AP_HAL::micros64());

    if (!test()) {
        if (!init_AKS16()) {
            hal.console->printf("AKS16::update(): could not init() MB4.\n");
            return;
        }
        else {
            hal.console->printf("AKS16::update(): MB4 alive test failed.\n");
            return;
        }
    }


}



bool AKS16::test() {

    if (sema == SEMA_NOT_AVAIL)
        return false;

    sema = SEMA_NOT_AVAIL;
    hal.scheduler->delay_microseconds(200);

    uint8_t res = mb4.mb4_read_param(&mb4.MB4_VERSION);
    hal.console->printf("AKS16: MB4 version %d, revision #%x\n", res, (unsigned int)mb4.mb4_read_param(&mb4.MB4_REVISION));
    if (res != 0x84) {
        if (res == 0)
            hal.console->printf("AKS16: MB4 not found\n");
        else
            hal.console->printf("AKS16: MB4 version bad = #%x\n", (unsigned int)res);
        sema = SEMA_AVAIL;
        return false;
    }

    if ((res = mb4.mb4_read_param(&mb4.MB4_CFGCH1)) != 1) {
        hal.console->printf("AKS16: MB4_CFGCH1 bad = #%x\n", (unsigned int)res);
        sema = SEMA_AVAIL;
        return false;
    }

    if ((res = mb4.mb4_read_param(&mb4.MB4_SLAVELOC5)) != 1) {
        hal.console->printf("AKS16: MB4_SLAVELOC5 bad = #%x\n", (unsigned int)res);
        sema = SEMA_AVAIL;
        return false;
    }
    res = mb4.mb4_read_param(&mb4.MB4_ENSCD1);
    uint8_t res2 = mb4.mb4_read_param(&mb4.MB4_ENSCD5);
    if (res != 1 || res2 != 1) {
        hal.console->printf("AKS16: MB4_ENSCD1 bad = #%x, MB4_ENSCD5 bad = #%x\n", (unsigned int)res, res2);
        sema = SEMA_AVAIL;
        return false;
    }

    StatusInformationF1 = mb4.mb4_read_param(&mb4.MB4_SVALID1);
    StatusInformationF5 = mb4.mb4_read_param(&mb4.MB4_SVALID5);
    if ((StatusInformationF1 & StatusInformationF5 & 0x01) != 0x01) {
        hal.console->printf("AKS16: SVALID1,5 bad F1 = #%x, F5 = #%x\n", (unsigned int)StatusInformationF1, StatusInformationF5);
        sema = SEMA_AVAIL;
        return false;
    }

    sema = SEMA_AVAIL;
    return true;
}

bool AKS16::init_AKS16() {

    sema = SEMA_NOT_AVAIL;
    hal.scheduler->delay(2);        // 2 mSec

//    hal.console->printf("AKS16::init_AKS16(): d1=%f, e1=%d\n", (float)_en1_degMin, (int)_en1_encMin);


    if (!mb4.mb4_init()) {
        hal.console->printf("AKS16::init_AKS16(): mb4_init() returned false\n");
        sema = SEMA_AVAIL;
        return false;
    }

    //BiSS/SSI Interface
    mb4.mb4_write_param(&mb4.MB4_CFGCH1, 0x01); //(BiSS C)
    mb4.mb4_write_param(&mb4.MB4_CFGCH2, 0x01); //(Not in use)    ????
    mb4.mb4_write_param(&mb4.MB4_CFGIF, 0x00); //(TTL=0, CMOS=1)
    mb4.mb4_write_param(&mb4.MB4_SLAVELOC5, 0x01); // (2 channels)

    mb4.mb4_write_param(&mb4.MB4_CLKENI, 0x01); // (internal clock=1)

    //Single-Cycle Data: Data channel configuration
    //  mb4.mb4_write_param(&mb4.MB4_ENSCD1, 0x01);
    mb4.mb4_write_param(&mb4.MB4_SCDLEN1, 0x19);
    mb4.mb4_write_param(&mb4.MB4_SELCRCS1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SCRCLEN1, 0x06);
    mb4.mb4_write_param(&mb4.MB4_SCRCSTART1,0x00);

    //  mb4.mb4_write_param(&mb4.MB4_ENSCD5, 0x01);
    mb4.mb4_write_param(&mb4.MB4_SCDLEN5, 0x19);
    mb4.mb4_write_param(&mb4.MB4_SELCRCS5, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SCRCLEN5, 0x06);
    mb4.mb4_write_param(&mb4.MB4_SCRCSTART5,0x00);

    //Frame Control: Master configuration
    mb4.mb4_write_param(&mb4.MB4_FREQS,0x04);
    mb4.mb4_write_param(&mb4.MB4_FREQAGS,0x80);   //0x63

    //Reset SVALID flags
    mb4.mb4_write_param(&mb4.MB4_SVALID1,0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5,0x00);
    //Start AGS
    mb4.mb4_write_param(&mb4.MB4_AGS,0x01);

    mb4.mb4_write_param(&mb4.MB4_ENSCD1, 0x01);
    mb4.mb4_write_param(&mb4.MB4_ENSCD5, 0x01);
    sema = SEMA_AVAIL;

    return true;
}

void AKS16::createBackProcess()
{
    // Starting the backend process
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> *devpp = mb4.get_devicepp();
    (*devpp)->register_periodic_callback(1250, FUNCTOR_BIND_MEMBER(&AKS16::update_encoders, void));

}

#define ERROR_VAL 999.999
#define PERCENT_EXTENDER  5.0

bool AKS16::checkconv_enc_vals(float &e1, float &e2)
{
    encStatus &= ~(SET_BIT(ENC1RANGE) | SET_BIT(ENC2RANGE));
    if ((encData1 < _en1_encMin * (1.0 - PERCENT_EXTENDER/100.0)) || (encData1 > _en1_encMax * (1.0 + PERCENT_EXTENDER/100.0))) {
        e1 = ERROR_VAL;
        encStatus |= SET_BIT(ENC1RANGE);
    }
    else
        e1 = _en1_degMin + (double)(encData1 - _en1_encMin) * (_en1_degMax - _en1_degMin) / (_en1_encMax - _en1_encMin);

    if (encData2 < (_en2_encMin * (1.0 - PERCENT_EXTENDER/100.0)) || (encData2 > _en2_encMax * (1.0 + PERCENT_EXTENDER/100.0))) {
        e2 = ERROR_VAL;
        encStatus |= SET_BIT(ENC2RANGE);
    }
    else
        e2 = _en2_degMin + (double)(encData2 - _en2_encMin) * (_en2_degMax - _en2_degMin) / (_en2_encMax - _en2_encMin);
    if (encStatus & (SET_BIT(ENC1RANGE) | SET_BIT(ENC2RANGE))) {
        hal.console->printf("c");
        return false;
    }
    return true;
}

void AKS16::printBytes(uint64_t v)
{
    hal.console->printf("v = %llx, composed of: ", v);
    for (int i = 0; i < 8; i++) {
        hal.console->printf("%x, ", (int)(v & 0xff));
        v = v >> 8;
    }

}

void AKS16::recover() {
    mb4.mb4_write_param(&mb4.MB4_BREAK, 0x01); //BREAK
    mb4.mb4_write_param(&mb4.MB4_AGS, 0x01); //Restart AGS

    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);

}

#define MAX_LOOP_USEC   400

void AKS16::update_encoders() {     // Backend process

    if (sema == SEMA_NOT_AVAIL)
        return;
    sema = SEMA_NOT_AVAIL;

    int StatusInformationF0;
    uint64_t startTime = AP_HAL::micros64();
    encStatus = 0;

    do {
        StatusInformationF0 = mb4.mb4_read_param(&mb4.MB4_EOT);
        if ((StatusInformationF0 & 0x01) != 0)
            break;
        if ((AP_HAL::micros64() - startTime) < MAX_LOOP_USEC) {
            recover();
            mb4.mb4_write_param(&mb4.MB4_HOLDBANK,0x00);
            encStatus |= SET_BIT(MB4_TIMEOUT);
            Write_MB4();    // Write to logger
            sema = SEMA_AVAIL;
            return ;
        }
    } while (true);
//    hal.console->printf("%lld,", AP_HAL::micros64() - startTime);
    mb4.mb4_write_param(&mb4.MB4_HOLDBANK,0x01);

    //Read and reset SVALID flags in Status Information register 0xF1
    StatusInformationF1 = mb4.mb4_read_param(&mb4.MB4_SVALID1);
    StatusInformationF5 = mb4.mb4_read_param(&mb4.MB4_SVALID5);
    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);

    StatusInformationF0_1 = mb4.mb4_read_param(&mb4.MB4_nSCDERR);
    StatusInformationF0_2 = mb4.mb4_read_param(&mb4.MB4_nDELAYERR);
    StatusInformationF0_3 = mb4.mb4_read_param(&mb4.MB4_nAGSERR);

//    encStatus &= ~(SET_BIT(SCDERR) | SET_BIT(DELAYERR) | SET_BIT(AGSERR) | SET_BIT(SVALID1) | SET_BIT(SVALID5));

    if ((StatusInformationF0_1 & StatusInformationF0_2 & StatusInformationF0_3) == 0x01) {
        //If Status ok but SVALID flag is not set, restart AGS
        if ((StatusInformationF1 & StatusInformationF5 & 0x01) == 0) {
            hal.console->printf("r");
            recover();
            if (!StatusInformationF1)
                encStatus |= SET_BIT(SVALID1);
            if (!StatusInformationF5)
                encStatus |= SET_BIT(SVALID5);
        }
        else {  //If Status ok and SVALID flag is set, read single-cycle data
            encData1 = mb4.mb4_read_param(&mb4.MB4_SCDATA1)>>6; //encData1
            encData2 = mb4.mb4_read_param(&mb4.MB4_SCDATA5)>>6; //encData2
//            hal.console->printf(".");
//            hal.console->printf("Reading 1,2: %X, %X\n", (int)encData1 , (int)encData2);
        }

    } else {
        if (!StatusInformationF0_1)
            encStatus |= SCDERR;
        if (!StatusInformationF0_2)
            encStatus |= DELAYERR;
        if (!StatusInformationF0_3)
            encStatus |= AGSERR;

        hal.console->printf("e");
        recover();
    }
    mb4.mb4_write_param(&mb4.MB4_HOLDBANK,0x00);

    checkconv_enc_vals(encDeg1, encDeg2);
    Write_MB4();    // Write to logger
//    hal.console->printf("Logging\n");

    //If Status not ok, check data channel configuration
    sema = SEMA_AVAIL;
    return ;
}

