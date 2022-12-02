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
            StatusInformationF1   : StatusInformationF1,
            StatusInformationF5   : StatusInformationF5,
            StatusInformationF0_1   : StatusInformationF0_1,
            StatusInformationF0_2   : StatusInformationF0_2,
            StatusInformationF0_3    : StatusInformationF0_3
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
