/*
  ESC Telemetry for the APD HV Pro ESC

  Protocol is here: https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
 */
#include "AP_APD_ESC.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_Math/definitions.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <string.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_APD_ESC *AP_APD_ESC::_singleton;

// constructor
AP_APD_ESC::AP_APD_ESC(void): uart(nullptr)
{
    _singleton = this;
}

void AP_APD_ESC::init() {

    gcs().send_text(MAV_SEVERITY_WARNING, "a");
    initialised = true;

    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager) {
        gcs().send_text(MAV_SEVERITY_WARNING, "b");
        uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_APD_ESC,0);
        if (uart) {
            uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_APD_ESC, 0));
        }
    }
}

#define MsgSize sizeof(received.packet)

void AP_APD_ESC::update() {
    if (uart == nullptr) {
        hal.console->printf("APD_ESC: uart is not initialized; ");
        return;
    }

    uint32_t n = uart->available();
//    hal.console->printf("APD_ESC: uart received %d chars", (int)n);

    if (n < MsgSize) {
        hal.console->printf("APD_ESC: received too few chars - %d bytes; ", (int)n);
        return;
    }

    while (n > 2 * MsgSize - 1) {
        uart->read();
        n--;
    }

    if (uart->read(received.bytes, n) == -1) {
        hal.console->printf("APD_ESC: Error reading %d bytes; ", (int)n);
        return;
    }

    while (n >= MsgSize) {
        if (received.bytes[n-1] == 0xff && received.bytes[n-2] == 0xff) {     // found the end of packet
            if (crc_fletcher16(&received.bytes[n-MsgSize], 18) == received.bytes[n-4] + received.bytes[n-3] * 255) {   // checksum pass assuming little endian TBTested
                if (n > MsgSize)
                    memmove(received.bytes, &received.bytes[n-MsgSize], sizeof(MsgSize-4));
                // valid packet, copy the data we need and reset length
                decoded.voltage = le16toh(received.packet.voltage) * 1e-2f;
                decoded.temperature = convert_temperature(le16toh(received.packet.temperature));
                decoded.current = le16toh(received.packet.bus_current) * (1 / 12.5f);
                decoded.rpm = le32toh(received.packet.erpm) / pole_count;
                decoded.power_rating_pct = le16toh(received.packet.motor_duty) * 1e-2f;
                len = 0;
                hal.console->printf("APD_ESC:  received voltage %f chars; ", (float)decoded.voltage);
                return;
            }
            else
                hal.console->printf("APD_ESC: Bad CRC; ");
        }
        else
            n--;
    }
    hal.console->printf("APD_ESC: Something went wrong - no data found. ");

//    while (n--) {
//        uint8_t b = uart->read();
//        received.bytes[len++] = b;
//
//        // check the packet size first
//        if ((size_t)len >= sizeof(received.packet)) {
//            gcs().send_text(MAV_SEVERITY_WARNING, "2");
//            // we have a full packet, check the stop byte
//            if (received.packet.stop == 65535) {
//                gcs().send_text(MAV_SEVERITY_WARNING, "3");
//                // valid stop byte, check the CRC
//                if (crc_fletcher16(received.bytes, 18) == received.packet.checksum) {
//                    gcs().send_text(MAV_SEVERITY_WARNING, "4");
//                    // valid packet, copy the data we need and reset length
//                    decoded.voltage = le16toh(received.packet.voltage) * 1e-2f;
//                    decoded.temperature = convert_temperature(le16toh(received.packet.temperature));
//                    decoded.current = le16toh(received.packet.bus_current) * (1 / 12.5f);
//                    decoded.rpm = le32toh(received.packet.erpm) / pole_count;
//                    decoded.power_rating_pct = le16toh(received.packet.motor_duty) * 1e-2f;
//                    len = 0;
//                    hal.console->printf("APD_ESC:  received voltage %f chars", (float)decoded.voltage);
//                } else {
//                    // we have an invalid packet, shift it back a byte
//                    shift_buffer();
//                }
//            } else {
//                // invalid stop byte, we've lost sync, shift the packet by 1 byte
//                shift_buffer();
//            }
//
//        }
//    }
}

// shift the decode buffer left by 1 byte, and rewind the progress
//void AP_APD_ESC::shift_buffer(void) {
//    memmove(received.bytes, received.bytes + 1, sizeof(received.bytes) - 1);
//    len--;
//}

// convert the raw ESC temperature to a useful value (in Kelvin)
// based on the 1.1 example C code found here https://docs.powerdrives.net/products/hv_pro/uart-telemetry-output
float AP_APD_ESC::convert_temperature(uint16_t raw) const {
    const float series_resistor     = 10000;
    const float nominal_resistance  = 10000;
    const float nominal_temperature = 25;
    const float b_coefficent        = 3455;


    const float Rntc = series_resistor / ((4096 / float(raw)) - 1);

    float temperature = Rntc / nominal_resistance;          // (R/Ro)
    temperature = logf(temperature);                        // ln(R/Ro)
    temperature /= b_coefficent;                            // 1/B * ln(R/Ro)
    temperature += 1 / C_TO_KELVIN(nominal_temperature); // + (1/To)
    temperature = 1 / temperature;                          // invert

    // the example code rejected anything below 0C, or above 200C, the 200C clamp makes some sense, the below 0C is harder to accept
    return temperature;
}

