/*
  ESC Telemetry for APD ESC.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>


class AP_APD_ESC {
public:
    AP_APD_ESC ();
    void init();
    void update();
    void sendMavlink(bool newData);

    CLASS_NO_COPY(AP_APD_ESC);

//    struct telem {
//        uint32_t error_count;
//        float voltage;
//        float current;
//        float temperature; // kelvin
//        int32_t rpm;
//        uint8_t power_rating_pct;
//    };

    struct esc_mav_telem {
        uint16_t voltage;
        uint16_t current;
        uint16_t totalCurrent;
        uint8_t  temperature; // kelvin
        uint16_t rpm;
        uint16_t count;
        uint8_t  status;
    };

    struct RPacket {
        uint16_t voltage;
        uint16_t temperature;
        int16_t bus_current;
        uint16_t reserved0;
        uint32_t erpm;
        uint16_t input_duty;
        uint16_t motor_duty;
        uint16_t reserved1;
        uint16_t checksum; // 16 bit fletcher checksum
        uint16_t stop; // should always be 65535 on a valid packet
    } ;

//#define MsgSize  sizeof(RPacket)
#define MsgSize 22




//    const telem &get_telem(void) {
//        return decoded;
//    }

    static AP_APD_ESC *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_APD_ESC *_singleton;

    // have we initialised the interface?
    bool initialised;

    AP_HAL::UARTDriver *uart;

//    static_assert(sizeof(received.packet) == sizeof(received.bytes), "The packet must be the same size as the raw buffer");

    struct esc_mav_telem decoded;

    float pole_count = 5*17.6;

    float convert_temperature(uint16_t raw) const;

    uint32_t _now;
    uint32_t _lastTime;

    // Mavlink data
    uint8_t _temperature[4] {0, 0, 0, 0};
    uint16_t _voltage[4] {0, 0, 0, 0};
    uint16_t _current[4]  {0, 0, 0, 0};
    uint16_t _totalcurrent[4] {0, 0, 0, 0};
    uint16_t _rpm[4] {0, 0, 0, 0};

};

