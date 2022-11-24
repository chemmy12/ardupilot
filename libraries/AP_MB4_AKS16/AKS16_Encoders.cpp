// the sensor communicates using SPI, so include the library:
#include "AP_MB4_AKS16_Enc.h"
#include "mb4_1sf_driver.h"


bool AKS16::init_AKS16() {

//  if (!mb4.mb4_init())
//      return false;
//
//  //BiSS/SSI Interface
//  mb4.mb4_write_param(&mb4.MB4_CFGCH1, 0x01); //(BiSS C)
//  mb4.mb4_write_param(&mb4.MB4_CFGCH2, 0x01); //(Not in use)
//  mb4.mb4_write_param(&mb4.MB4_CFGIF, 0x00); //(TTL)
//  mb4.mb4_write_param(&mb4.MB4_SLAVELOC5, 0x01);
//
//
//  //Single-Cycle Data: Data channel configuration
//  mb4.mb4_write_param(&mb4.MB4_ENSCD1, 0x01);
//  mb4.mb4_write_param(&mb4.MB4_SCDLEN1, 0x19);
//  mb4.mb4_write_param(&mb4.MB4_SELCRCS1, 0x00);
//  mb4.mb4_write_param(&mb4.MB4_SCRCLEN1, 0x06);
//  mb4.mb4_write_param(&mb4.MB4_SCRCSTART1,0x00);
//
//  mb4.mb4_write_param(&mb4.MB4_ENSCD5, 0x01);
//  mb4.mb4_write_param(&mb4.MB4_SCDLEN5, 0x19);
//  mb4.mb4_write_param(&mb4.MB4_SELCRCS5, 0x00);
//  mb4.mb4_write_param(&mb4.MB4_SCRCLEN5, 0x06);
//  mb4.mb4_write_param(&mb4.MB4_SCRCSTART5,0x00);
//
//
//  //Frame Control: Master configuration
//  mb4.mb4_write_param(&mb4.MB4_FREQS,0x04);
//  mb4.mb4_write_param(&mb4.MB4_FREQAGS,0x63);
//
//
//
//  //Reset SVALID flags
//  mb4.mb4_write_param(&mb4.MB4_SVALID1,0x00);
//  mb4.mb4_write_param(&mb4.MB4_SVALID5,0x00);
//  //Start AGS
//  mb4.mb4_write_param(&mb4.MB4_AGS,0x01);

  return true;
}

bool AKS16::update_encoders() {


//  currentTime = millis();
//  if(currentTime > displayTime)
//  {
//    displayTime = currentTime + 10;
    //Reset SVALID flags
    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);
    //Start AGS
    mb4.mb4_write_param(&mb4.MB4_AGS, 0x01);;

    //Read Status Information register 0xF0, wait for end of transmission EOT=1
    do {
        //hal.console->printf("3\n");
        StatusInformationF0 = mb4.mb4_read_param(&mb4.MB4_EOT);
    } while ((StatusInformationF0 & 0x01) == 0);

    //mb4.mb4_write_param(&mb4.MB4_HOLDBANK,0x01);

    //Read and reset SVALID flags in Status Information register 0xF1
    StatusInformationF1 = mb4.mb4_read_param(&mb4.MB4_SVALID1);
    StatusInformationF5 = mb4.mb4_read_param(&mb4.MB4_SVALID5);
    mb4.mb4_write_param(&mb4.MB4_SVALID1, 0x00);
    mb4.mb4_write_param(&mb4.MB4_SVALID5, 0x00);

    SCDATA1 = mb4.mb4_read_param(&mb4.MB4_SCDATA1); //SCDATA1
    //OUT1 = (uint32_t) SCDATA1;
    //OUT2 = (uint32_t) (SCDATA1 >> 32);
    //hal.console->printf("Data1: %04lx, %04lx\n", OUT1, OUT2);


    StatusInformationF0_1 = mb4.mb4_read_param(&mb4.MB4_nSCDERR);
    StatusInformationF0_2 = mb4.mb4_read_param(&mb4.MB4_nDELAYERR);
    StatusInformationF0_3 = mb4.mb4_read_param(&mb4.MB4_nAGSERR);
    //hal.console->printf("StatusInformationF0: %d, F1: %d", F0_1: %d F0_2: %d, F0_3: %d\n",
    // StatusInformationF0,
    // StatusInformationF1,
    // StatusInformationF0_1,
    // StatusInformationF0_2,
    // StatusInformationF0_3);

    if ((StatusInformationF0_1 & StatusInformationF0_2 & StatusInformationF0_3) == 0x01) {
        //hal.console->printf("5\n");
        //If Status ok but SVALID flag is not set, restart AGS
        if ((StatusInformationF1 & StatusInformationF5 & 0x01) == 0) {
//          hal.console->printf("6\n");
            mb4.mb4_write_param(&mb4.MB4_BREAK, 0x01); //BREAK
            mb4.mb4_write_param(&mb4.MB4_AGS, 0x01); //Restart AGS
        }

            //If Status ok and SVALID flag is set, read single-cycle data
        else {
            //hal.console->printf("7\n");
            SCDATA1 = mb4.mb4_read_param(&mb4.MB4_SCDATA1); //SCDATA1
            SCDATA5 = mb4.mb4_read_param(&mb4.MB4_SCDATA5); //SCDATA1
          hal.console->printf("(%ld, %ld)\n", SCDATA1 >> 6, SCDATA5 >> 6);
        }
        mb4.mb4_write_param(&mb4.MB4_HOLDBANK, 0x00);
    } else {
    }
    //If Status not ok, check data channel configuration
    return true;
//}
}
//
//void mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize) {
//    digitalWrite(chipSelectPin, LOW);
//
//    for (uint8_t i = 0; i < datasize; i++)
//        data_rx[i] = SPI.transfer(data_tx[i]);
//
//    digitalWrite(chipSelectPin, HIGH);
//}
