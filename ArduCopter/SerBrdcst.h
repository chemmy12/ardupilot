//
// Created by chemmy on 5/29/23.
//

#ifndef ARDUPILOT_SERIAL_SERBRDCST_H
#define ARDUPILOT_SERIAL_SERBRDCST_H

#include "AP_HAL/AP_HAL.h"
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <stdint.h>


#define BROADCAST_PERIOD_MS     100
#define TIMEOUT_MS  1000

struct dataBlock {
    uint16_t    header;
    int16_t     pitch;
    int16_t     roll;
//    uint8_t     status;    // A spare.
    uint16_t    crc;
};

#define SIZE_OF_DATA    (sizeof(dataBlock) - sizeof(uint16_t))


class SerBrdcst {
    enum ComState {NOCOMM, SENDER, RECEIVER};
public:
    SerBrdcst();
    ~SerBrdcst() {};
    bool init();
    void update() {}
    bool sendData(int16_t roll, int16_t pitch);
    bool recvData(int16_t &roll, int16_t &pitch);

private:
    ComState comState;
    // pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr;
    char _buf[sizeof(dataBlock)*2];

    bool checkCrc(dataBlock *db);

};


#endif //ARDUPILOT_SERIAL_SERBRDCST_H
