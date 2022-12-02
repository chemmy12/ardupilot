// the sensor communicates using SPI, so include the library:
#include "AKS16.h"
#include "mb4_1sf_driver.h"

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
        notYetInit(true)
{
}


void AKS16::update()
{

    if (notYetInit) {    // happens only once at boot
        init_AKS16();
        createBackProcess();
        notYetInit = false;
    }

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
    uint8_t res = mb4.mb4_read_param(&mb4.MB4_VERSION);
//    hal.console->printf("AKS16: MB4 version=%d\n", res);
//    hal.console->printf("AKS16: MB4 revision #%x\n", (unsigned int)mb4.mb4_read_param(&mb4.MB4_REVISION));
    if (res != 0x84) {
        if (res == 0)
            hal.console->printf("AKS16: MB4 not found\n");
        else
            hal.console->printf("AKS16: MB4 version bad = #%x\n", (unsigned int)res);
        return false;
    }

    if ((res = mb4.mb4_read_param(&mb4.MB4_CFGCH1)) != 1) {
        hal.console->printf("AKS16: MB4_CFGCH1 bad = #%x\n", (unsigned int)res);
        return false;
    }

    if ((res = mb4.mb4_read_param(&mb4.MB4_SLAVELOC5)) != 1) {
        hal.console->printf("AKS16: MB4_SLAVELOC5 bad = #%x\n", (unsigned int)res);
        return false;
    }

    return true;
}

bool AKS16::init_AKS16() {

    hal.console->printf("AKS16::init_AKS16(): d1=%f, e1=%d\n", (float)_en1_degMin, (int)_en1_encMin);


    if (!mb4.mb4_init())
      return false;


  //BiSS/SSI Interface
  mb4.mb4_write_param(&mb4.MB4_CFGCH1, 0x01); //(BiSS C)
  mb4.mb4_write_param(&mb4.MB4_CFGCH2, 0x01); //(Not in use)    ????
  mb4.mb4_write_param(&mb4.MB4_CFGIF, 0x00); //(TTL=0, CMOS=1)
  mb4.mb4_write_param(&mb4.MB4_SLAVELOC5, 0x01); // (2 channels)

  //Single-Cycle Data: Data channel configuration
  mb4.mb4_write_param(&mb4.MB4_ENSCD1, 0x01);
  mb4.mb4_write_param(&mb4.MB4_SCDLEN1, 0x19);
  mb4.mb4_write_param(&mb4.MB4_SELCRCS1, 0x00);
  mb4.mb4_write_param(&mb4.MB4_SCRCLEN1, 0x06);
  mb4.mb4_write_param(&mb4.MB4_SCRCSTART1,0x00);

  mb4.mb4_write_param(&mb4.MB4_ENSCD5, 0x01);
  mb4.mb4_write_param(&mb4.MB4_SCDLEN5, 0x19);
  mb4.mb4_write_param(&mb4.MB4_SELCRCS5, 0x00);
  mb4.mb4_write_param(&mb4.MB4_SCRCLEN5, 0x06);
  mb4.mb4_write_param(&mb4.MB4_SCRCSTART5,0x00);

  //Frame Control: Master configuration
  mb4.mb4_write_param(&mb4.MB4_FREQS,0x04);
  mb4.mb4_write_param(&mb4.MB4_FREQAGS,0x63);

  //Reset SVALID flags
  mb4.mb4_write_param(&mb4.MB4_SVALID1,0x00);
  mb4.mb4_write_param(&mb4.MB4_SVALID5,0x00);
  //Start AGS
  mb4.mb4_write_param(&mb4.MB4_AGS,0x01);

    return true;
}

void AKS16::createBackProcess()
{
    // Starting the backend process
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> *devpp = mb4.get_devicepp();
    (*devpp)->register_periodic_callback(4000, FUNCTOR_BIND_MEMBER(&AKS16::update_encoders, void));

}

#define ERROR_VAL 999.999

bool AKS16::checkconv_enc_vals(float &e1, float &e2)
{
    if (encData1 < _en1_encMin || encData1 > _en1_encMax)
        e1 = ERROR_VAL;
    else
        e1 = _en1_degMin + (double)(encData1 - _en1_encMin) * (_en1_degMax - _en1_degMin) / (_en1_encMax - _en1_encMin);

    if (encData2 < _en2_encMin || encData2 > _en2_encMax)
        e2 = ERROR_VAL;
    else
        e2 = _en2_degMin + (double)(encData2 - _en2_encMin) * (_en2_degMax - _en2_degMin) / (_en2_encMax - _en2_encMin);
    if (e1 > (ERROR_VAL-1) && e2 > (ERROR_VAL-1))
        return false;
    return true;
}


void AKS16::update_encoders() {     // Backend process
//    hal.console->printf("AKS16::update_encoders();\n");

//    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
//    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);
//    //Start AGS
//    mb4.mb4_write_param(&mb4.MB4_AGS, 0x01);;
//
//    //Read Status Information register 0xF0, wait for end of transmission EOT=1
//#ifdef WAIT_LOOP
//    do {
//        //hal.console->printf("3\n");
//        StatusInformationF0 = mb4.mb4_read_param(&mb4.MB4_EOT);
//    } while ((StatusInformationF0 & 0x01) == 0);
//#endif
//    //mb4.mb4_write_param(&mb4.MB4_HOLDBANK,0x01);

    //Read and reset SVALID flags in Status Information register 0xF1
    StatusInformationF1 = mb4.mb4_read_param(&mb4.MB4_SVALID1);
    StatusInformationF5 = mb4.mb4_read_param(&mb4.MB4_SVALID5);
    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);

//    encData1 = mb4.mb4_read_param(&mb4.MB4_SCDATA1); //encData1
//    encDeg1 = (uint32_t) encData1;
//    SCDATA5 = mb4.mb4_read_param(&mb4.MB4_SCDATA5); //SCDATA5
//    encDeg2 = (uint32_t) SCDATA5;
//    hal.console->printf("Data1,5: %04ld, %04ld\n", encDeg1, encDeg2);


    StatusInformationF0_1 = mb4.mb4_read_param(&mb4.MB4_nSCDERR);
    StatusInformationF0_2 = mb4.mb4_read_param(&mb4.MB4_nDELAYERR);
    StatusInformationF0_3 = mb4.mb4_read_param(&mb4.MB4_nAGSERR);
//    hal.console->printf("StatusInformationF0: F1: %d, F0_1: %d F0_2: %d, F0_3: %d\n",
//         StatusInformationF1,
//         StatusInformationF0_1,
//         StatusInformationF0_2,
//         StatusInformationF0_3);

    if ((StatusInformationF0_1 & StatusInformationF0_2 & StatusInformationF0_3) == 0x01) {
//        hal.console->printf("5\n");
        //If Status ok but SVALID flag is not set, restart AGS
        if ((StatusInformationF1 & StatusInformationF5 & 0x01) == 0) {
            hal.console->printf("update_encoders: bad F1 || F5 regs\n");
            mb4.mb4_write_param(&mb4.MB4_BREAK, 0x01); //BREAK
            mb4.mb4_write_param(&mb4.MB4_AGS, 0x01); //Restart AGS
            return;
        }

            //If Status ok and SVALID flag is set, read single-cycle data
        else {
//            hal.console->printf("7\n");
            encData1 = mb4.mb4_read_param(&mb4.MB4_SCDATA1); //encData1
            encData2 = mb4.mb4_read_param(&mb4.MB4_SCDATA5); //encData2
            hal.console->printf("Reading: %ld, %ld\n", encData1 >> 6, encData2 >> 6);
        }

//        mb4.mb4_write_param(&mb4.MB4_HOLDBANK, 0x00);
    } else {
    }
    checkconv_enc_vals(encDeg1, encDeg2);
    Write_MB4();

    //If Status not ok, check data channel configuration
    return ;

//}
}
