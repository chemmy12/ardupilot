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

    CLASS_NO_COPY(AP_APD_ESC);

    struct telem {
        uint32_t error_count;
        float voltage;
        float current;
        float temperature; // kelvin
        int32_t rpm;
        uint8_t power_rating_pct;
    };

    const telem &get_telem(void) {
        return decoded;
    }

    static AP_APD_ESC *get_singleton(void) {
        return _singleton;
    }

private:
    static AP_APD_ESC *_singleton;

    // have we initialised the interface?
    bool initialised;

    AP_HAL::UARTDriver *uart;

    union {
        struct {
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
        } packet;
        uint8_t bytes[22*2];
    } received;
//    static_assert(sizeof(received.packet) == sizeof(received.bytes), "The packet must be the same size as the raw buffer");

    uint8_t len;

    struct telem decoded;

    float pole_count = 5*17.6;

    float convert_temperature(uint16_t raw) const;
    void shift_buffer(void);
};

