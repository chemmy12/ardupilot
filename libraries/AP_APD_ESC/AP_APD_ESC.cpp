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
        if (n != 0)
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
                decoded.voltage = le16toh(received.packet.voltage);
                decoded.temperature = ((uint8_t)convert_temperature(le16toh(received.packet.temperature))) & 0xff;
                decoded.current = (uint16_t)(le16toh(received.packet.bus_current) * (1 / 12.5f));
                decoded.rpm = (uint16_t)(le32toh(received.packet.erpm) / pole_count);
                decoded.totalCurrent = le16toh(received.packet.motor_duty);
                len = 0;
                hal.console->printf("APD_ESC:  received voltage %d chars; ", decoded.voltage);

                sendMavlink();

                return;
            }
            else
                hal.console->printf("APD_ESC: Bad CRC; ");
        }
        else
            n--;
    }
    hal.console->printf("APD_ESC: Something went wrong - no data found. ");

}


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

void AP_APD_ESC::sendMavlink()
{
//    mavlink_msg_esc_telemetry_1_to_4_send(mavlink_channel_t chan, const uint8_t *temperature, const uint16_t *voltage, const uint16_t *current, const uint16_t *totalcurrent, const uint16_t *rpm, const uint16_t *count)
//    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) mav_chan, temperature, voltage, current, totalcurrent, rpm, count);
    const uint8_t temperature[4]  {decoded.temperature, 0, 0, 0};
    const uint16_t voltage[4] {decoded.voltage, 346, 347, 348};
    const uint16_t current[4]  {decoded.current, 457, 458, 459};
    const uint16_t totalcurrent[4] {decoded.totalCurrent, 568, 569, 570};
    const uint16_t rpm[4] {decoded.rpm, 679, 680, 681};
    static uint16_t count[4] {0, 790, 791, 792};
    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) 0, temperature, voltage, current, totalcurrent, rpm, count);
    count[0]++;

    hal.console->printf("APD_ESC: Sent Mavlink message??? count0=%d\n", count[0]);

}