//
// Created by chemmy on 11/30/22.
//

#include "AP_MB4_AKS16/AP_MB4_AKS16_Enc.h"
#include <AP_Logger/AP_Logger.h>

//struct PACKED log_AKS {
//        LOG_PACKET_HEADER;
//        uint64_t time_us;
//        float out1;
//        float out2;
//        uint8_t StatusInformationF1;
//        uint8_t StatusInformationF5;
//        uint8_t StatusInformationF0_1;
//        uint8_t StatusInformationF0_2;
//        uint8_t StatusInformationF0_3;
//}

// Write an AHRS2 packet
void AKS16::Write_MB4() // const
{

    const struct log_AKS pkt{
            LOG_PACKET_HEADER_INIT(LOG_AKS16_MSG),
            time_us : AP_HAL::micros64(),
            out1  : (float)OUT1,
            out2  : (float)OUT2,
            StatusInformationF1   : StatusInformationF1,
            StatusInformationF5   : StatusInformationF5,
            StatusInformationF0_1   : StatusInformationF0_1,
            StatusInformationF0_2   : StatusInformationF0_2,
            StatusInformationF0_3    : StatusInformationF0_3
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
