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
#include <AP_Logger/AP_Logger.h>

//#define APD_DEBUG


extern const AP_HAL::HAL& hal;

AP_APD_ESC *AP_APD_ESC::_singleton;

// constructor
AP_APD_ESC::AP_APD_ESC(void): uart(nullptr)
{
    _singleton = this;
}

void AP_APD_ESC::init() {

#ifdef APD_DEBUG
    gcs().send_text(MAV_SEVERITY_WARNING, "APD_ESC::init()");
#endif
    initialised = true;

    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (serial_manager) {
//        gcs().send_text(MAV_SEVERITY_WARNING, "b");
        uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_APD_ESC,0);
        if (uart) {
            uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
            uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_APD_ESC, 0));
        }
        else
            hal.console->printf("APD_ESC::init(): uart is not initialized;\n");
    }
}

extern void printn(uint8_t *p, int n, const char* h);

void AP_APD_ESC::update() {
    union {
        uint8_t rdata[MsgSize * 2];
        RPacket packet;
    };


    if (uart == nullptr) {
#ifdef APD_DEBUG
        hal.console->printf("APD_ESC: uart is not initialized; ");
#endif
        return;
    }

    uint32_t n = uart->available();
#ifdef APD_DEBUG
    hal.console->printf("APD_ESC: uart received %d chars", (int)n);
#endif
    if (n < MsgSize) {
#ifdef APD_DEBUG
        if (n != 0)
            hal.console->printf("APD_ESC: received too few chars - %d bytes; ", (int)n);
#endif
        sendMavlink(false);
        return;
    }

    while (n > 2 * MsgSize - 1) {
        uart->read();
        n--;
    }
#ifdef APD_DEBUG
    hal.console->printf("APD_ESC: MsgSize=%d, n=%d\n", MsgSize, (int)n);
#endif

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
                decoded.temperature = (uint8_t)convert_temperature(le16toh(packet.temperature));
                decoded.current = (uint16_t)(le16toh(packet.bus_current) * (1 / 12.5f));
                decoded.rpm = (uint16_t)(le32toh(packet.erpm) / pole_count);
                decoded.totalCurrent = le16toh(packet.motor_duty);

                decoded.status = le16toh(packet.reserved1) & 0xff;

//                hal.console->printf("APD_ESC:  received voltage %d volt; ", decoded.voltage);

                sendMavlink(true);

                while(uart->available() > 0)   // clean buffer if  needed.
                    uart->read();

                return;
            }
            else {
//                hal.console->printf("APD_ESC: Bad CRC; fletch=%04X, crc=%04X\n", crc_fletcher16((const uint8_t *)&packet, 18), packet.checksum);
                sendMavlink(false);
                return;
            }
        }
        else {
            n--;
            hal.console->printf("-");
        }
    }
    sendMavlink(false);
//    hal.console->printf("APD_ESC: Something went wrong - no data found. ");

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
    temperature = 1 / temperature - 273.15;                          // invert .... and convert to Celsius

    // the example code rejected anything below 0C, or above 200C, the 200C clamp makes some sense, the below 0C is harder to accept
    return temperature;
}

#define DATA_RESET_MS   500
#define SEND_MS_PERIOD  250

void AP_APD_ESC::sendMavlink(bool newData)
{
    static uint16_t counter = 0;
    static uint32_t lastSend = 0;

    _now = AP_HAL::millis();

    if (newData) {
        AP::logger().Write("APDE", "TimeUS,status,temp,volt,curr,tcurr,rpm",
                           "s-OvAAq", // units: seconds, none, cd, cd
                           "F------", // mult: 1e-6, 1, 1e-2, 1e-2
                           "QBBfffH", // format: uint64_t, uint16_t, int16_t, int16_t
                           AP_HAL::micros64(), decoded.status, decoded.temperature, decoded.voltage / 100.0,
                           decoded.current / 100.0, decoded.totalCurrent / 100.0, decoded.rpm);

        _lastTime = _now;
    }

    if (_now - lastSend < SEND_MS_PERIOD)
        return;
    lastSend = _now;

    if (_now - _lastTime > DATA_RESET_MS) {
        _temperature[0] = 0;
        _voltage[0] = 0;
        _current[0] = 0;
        _totalcurrent[0] = 0;
        _rpm[0] = 0;
    }
    else {
        _temperature[0] = decoded.temperature;
        _temperature[1] = decoded.status;
        _voltage[0] = decoded.voltage;
        _current[0] = decoded.current;
        _totalcurrent[0] = decoded.totalCurrent;
        _rpm[0] = decoded.rpm;

    }
    const uint16_t count[4] {counter, counter, counter, counter++};

    for (int i = 0; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        if (gcs().chan(i)) {
            hal.console->printf("APD_ESC sending Mavlink to channel offset %d\n", i);
            mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t) i, _temperature, _voltage, _current,
                                                  _totalcurrent, _rpm, count);
        } else
            break;
    }

#ifdef APD_DEBUG
    hal.console->printf("APD_ESC: Sent Mavlink message??? count0=%d\n", count[0]);
#endif
}

void printn(uint8_t *p, int n, const char* h)
{
    hal.console->printf("APD_ESC:%s ", h);
    for (int i = 0; i < n; i++)
        hal.console->printf("%d(%02X) ", i, (unsigned int)(*p++));
    hal.console->printf("\n");
}

