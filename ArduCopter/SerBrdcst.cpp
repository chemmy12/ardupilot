//
// Created by chemmy on 5/29/23.
//

#include "SerBrdcst.h"
#include "Copter.h"

extern const AP_HAL::HAL &hal;

#define CRC_SEED    0xaaaa
#define HEADER      0x0102
#define HEADER_LOW  (HEADER & 0xff)
#define HEADER_HIGH ((HEADER >> 8) & 0xff)


SerBrdcst::SerBrdcst(): _uart(nullptr)
{
}

bool SerBrdcst::init()
{
    hal.console->printf("SerBrdcst::init()\n");
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_BodyAngles, 0);
    if (_uart == nullptr) {
        return false;
    }

    _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_BodyAngles, 0));

    return true;
}

#define maxMSOfFreeze   100
void SerBrdcst::recvUpdate()
{
    int16_t r,p;
    static int16_t oldr = 0, oldp = 0;
    if (recvData(r, p)) {
        /* hal.console->printf("SerBrdcst: Received r=%f, p=%f\n", r/100.0, p/100.0) */{};
        if (r != oldr || p != oldp) {
            _timeLastGoodData = AP_HAL::millis();
            _roll = r;
            _pitch = p;
            _status = true;
        }
    }
    else if (AP_HAL::millis() - _timeLastGoodData > maxMSOfFreeze) {
        _status = false;
        _pitch = 0;
        _roll = 0;
    }
    AP::logger().Write("BANG", "TimeUS,stat,roll,pitch",
                       "s-dd", // units: seconds, none, cd, cd
                       "F---", // mult: 1e-6, 1, 1e-2, 1e-2
                       "QHff", // format: uint64_t, uint16_t, int16_t, int16_t
                       AP_HAL::micros64(), _status, _roll/100.0, _pitch/100.0);
}

bool SerBrdcst::getRPS(int16_t &r, int16_t &p)
{
    r = _roll;
    p = _pitch;
    return _status;
}
void SerBrdcst::sendUpdate()
{
    assert(1);
//    int16_t pitch = copter.ahrs.pitch_sensor;     // units in centi-degrees
//    int16_t roll = copter.ahrs.roll_sensor;
//    sendData(roll, pitch);
}


// read the from the sensor
bool SerBrdcst::sendData(int16_t roll, int16_t pitch)
{
    if (_uart == nullptr) {
        hal.console->printf("SerBrdcst::sendData: _uart is nullptr\n");
//        init();
        return false;
    }

    uint32_t now = AP_HAL::millis();
    static uint32_t _last_update_ms = 0;

    struct dataBlock buf;
    uint8_t * cp = (uint8_t*)(&buf);

    buf.header = HEADER;
    buf.pitch = pitch;
    buf.roll = roll;
    buf.crc = CRC_SEED;
    for (int i = 0; i < SIZE_OF_DATA; i++, cp++)
        buf.crc += *cp;

//    hal.console->printf("SerBrdcst::sendData(): R=%2.2f P=%2.2f", roll, pitch);
    _uart->write((uint8_t *)(&buf), sizeof(buf));

    bool success = (now - _last_update_ms) < TIMEOUT_MS;
    _last_update_ms = now;
    return success;
}

bool SerBrdcst::recvData(int16_t &roll, int16_t &pitch)
{
    roll = pitch = 0;

    if (_uart == nullptr) {
        return false;
    }

    uint32_t nbytes;
    // clear excessive data
    for(nbytes = _uart->available(); nbytes >= sizeof(dataBlock)*2; nbytes--) {
        _uart->read();
    }

    // if not enough data - ignore until next time
    if (nbytes < sizeof(dataBlock)) {
//        if (nbytes > 0)
//            hal.console->printf("SerBrdcst received only %lu bytes\n", nbytes);
        return false;
    }
    uint8_t buf[sizeof(dataBlock)*2];
    uint8_t *cp = buf;
    // fill data into buffer
    for (int i = 0; i < nbytes /* && i < sizeof(buf)*/; i++) {
        *cp++ = _uart->read();
    }
    // search backward for potential header
    dataBlock db;
    for (int i = nbytes - sizeof(dataBlock); i >= 0; i--) {
//        hal.console->printf("SerBrdcst::recvData: checking on %d bytes starting at %d\n", nbytes, i);
        if (buf[i] == HEADER_LOW) {    // potential header
//            hal.console->printf("SerBrdcst::recvData: byte %d is 0x02\n", i);
            if (buf[i+1] == HEADER_HIGH) {
//                hal.console->printf("SerBrdcst::recvData: byte %d is 0x01\n", i+1);
                memcpy(&db, &buf[i], sizeof(dataBlock));
                if (checkCrc(&db)) {
                    roll = db.roll;
                    pitch = db.pitch;
//                    hal.console->printf("SerBrdcst::recvData: R=%2.2f P=%2.2f\n", roll/100.0, pitch/100.0);
                    return true;
                }
            }
//            else
//                hal.console->printf("SerBrdcst::recvData: byte %d+1 is %X (bad)\n", i, buf[i+1]);
        }
    }
//    hal.console->printf("SerBrdcst::recvData: no header found. db size=%d\n", sizeof(dataBlock));
    return false;
}

bool SerBrdcst::checkCrc(dataBlock *db)
{
    uint16_t crc = CRC_SEED;
    uint8_t *cp = (uint8_t*)db;
    for (int i = 0; i < SIZE_OF_DATA; i++, cp++)
        crc += *cp;
    return (crc == db->crc) ? true : false;
}