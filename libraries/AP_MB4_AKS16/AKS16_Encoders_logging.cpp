//
// Created by chemmy on 11/30/22.
//

#include "AP_MB4_AKS16/AKS16.h"
#include <AP_Logger/AP_Logger.h>

// Write an AHRS2 packet
void AKS16::Write_MB4() // const
{

    const struct log_AKS pkt{
            LOG_PACKET_HEADER_INIT(LOG_AKS16_MSG),
            time_us : AP_HAL::micros64(),
            enc1: encData1,
            enc2: encData2,
            encDeg1  : encDeg1,
            encDeg2  : encDeg2,
            encDegFilt1  : lowPassEnc1.get(),
            encDegFilt2  : lowPassEnc2.get(),
            encStatus   : encStatus
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

void AKS16::slow_write_mb4(short miligap)
{
    static int last = 0;

    int now = AP_HAL::millis();
    if (last == 0 || now - last > miligap) {
        last = now;
        Write_MB4();
    }
}
