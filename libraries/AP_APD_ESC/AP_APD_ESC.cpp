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

//    gcs().send_text(MAV_SEVERITY_WARNING, "a");
    initialised = true;

    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager) {
//        gcs().send_text(MAV_SEVERITY_WARNING, "b");
        uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_APD_ESC,0);
        if (uart) {
            uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_APD_ESC, 0));
        }
    }
}


extern void printn(uint8_t *p, int n, const char* h);

void AP_APD_ESC::update() {
    union {
        uint8_t rdata[MsgSize * 2];
        RPacket packet;
    };


    if (uart == nullptr) {
        hal.console->printf("APD_ESC: uart is not initialized; ");
        return;
    }

    uint32_t n = uart->available();
    hal.console->printf("APD_ESC: uart received %d chars", (int)n);

    if (n < MsgSize) {
        if (n != 0)
            hal.console->printf("APD_ESC: received too few chars - %d bytes; ", (int)n);
        return;
    }

    while (n > 2 * MsgSize - 1) {
        uart->read();
        n--;
    }

//    hal.console->printf("APD_ESC: MsgSize=%d, n=%d\n", MsgSize, (int)n);

    for (int i = 0; i < n; i++)      // copy received data to buffer
        rdata[i] = uart->read();

//    printn(rdata, n, "dump recv buf:");

    while (n >= MsgSize) {
        if (rdata[n-1] == 0xff && rdata[n-2] == 0xff) {     // found the end of packet
            if (n > MsgSize) {
                memmove(&packet, &rdata[n - MsgSize], MsgSize);
                n = 0;  // don't iterate any more.... for now. need to be improved
            }
//            printn(rdata, 22, "dump final buf:");
            if (crc_fletcher16((const uint8_t *)&packet, 18) == packet.checksum) {   // checksum pass assuming little endian TBTested
                // valid packet, copy the data we need and reset length
                decoded.voltage = le16toh(packet.voltage);
                decoded.temperature = (uint8_t)(convert_temperature(le16toh(packet.temperature)) - 273);
                decoded.current = (uint16_t)(le16toh(packet.bus_current) * (1 / 12.5f));
                decoded.rpm = (uint16_t)(le32toh(packet.erpm) / pole_count);
                decoded.totalCurrent = le16toh(packet.motor_duty);

//                hal.console->printf("APD_ESC:  received voltage %d volt; ", decoded.voltage);

                sendMavlink();

                while(uart->available() > 0)   // clean buffer if  needed.
                    uart->read();

                return;
            }
            else {
                hal.console->printf("APD_ESC: Bad CRC; fletch=%04X, crc=%04X\n", crc_fletcher16((const uint8_t *)&packet, 18), packet.checksum);
                return;
            }
        }
        else {
            n--;
            hal.console->printf("-");
        }
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
    const uint16_t voltage[4] {decoded.voltage, 0,0,0};
    const uint16_t current[4]  {decoded.current, 0,0,0};
    const uint16_t totalcurrent[4] {decoded.totalCurrent, 0,0,0};
    const uint16_t rpm[4] {decoded.rpm, 0,0,0};
    static uint16_t count[4] {0, 0,0,0};
    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) 0, temperature, voltage, current, totalcurrent, rpm, count);
    count[0]++;

    hal.console->printf("APD_ESC: Sent Mavlink message??? count0=%d\n", count[0]);

}

void printn(uint8_t *p, int n, const char* h)
{
    hal.console->printf("APD_ESC:%s ", h);
    for (int i = 0; i < n; i++)
        hal.console->printf("%d(%02X) ", i, (unsigned int)(*p++));
    hal.console->printf("\n");
}