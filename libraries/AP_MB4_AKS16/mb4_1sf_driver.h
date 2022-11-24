/**
 ******************************************************************************
 * @file    mb4_1sf_driver.h
 * @author  iC-Haus GmbH
 * @version 1.1.0
 * @note Designed according to iC-MB4 datasheet release D2 for chip revision Y.
 ******************************************************************************
 * @attention
 *
 *	Software and its documentation is provided by iC-Haus GmbH or contributors "AS IS" and is
 *	subject to the ZVEI General Conditions for the Supply of Products and Services with iC-Haus
 *	amendments and the ZVEI Software clause with iC-Haus amendments (http://www.ichaus.de/EULA).
 *
 ******************************************************************************
 */

#ifndef MB4_1SF_DRIVER_H
#define MB4_1SF_DRIVER_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "stdint.h"

#include "AP_HAL/AP_HAL.h"

extern const AP_HAL::HAL& hal;

/**
 * @defgroup Wrapper_Functions
 * @brief Functions declared as `extern` are to be defined by the user according to the individual MCU.
 *
 * @note The definition of those functions is mandatory for proper use of this driver.
 * @{
 */
/**
 * @brief This function is a wrapper to be defined with the hardware-related SPI-transmit-receive-function.
 *
 * @param data_tx is a pointer to the data buffer that is transmitted.
 * @param data_rx is a pointer to the buffer the received data is written to.
 * @param datasize is the length of all bytes transmitted.
 * @retval None
 */
//extern void mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize);
/**
 * @}
 */

/**
 * @defgroup MB4_Parameters
 * @{
 */
/**
 * @addtogroup MB4_Parameters_Struct
 * @brief Structure implemented to represent iC-MB4 parameters.
 * @{
 */
struct mb4_param {
	uint8_t addr;
	uint8_t pos;
	uint8_t len;
};
/**
 * @}
 */

/**
 * @addtogroup MB4_Parameters_List
 * @brief List of parameters according to iC-MB4 register map represented as @ref MB4_Parameters_Struct.
 * @{
 */

class MB4 {
private:
    enum MB4_OPCODE {
        MB4_OPCODE_WRITE_REGISTERS = 0x02,
        MB4_OPCODE_READ_REGISTERS = 0x03,
        MB4_OPCODE_READ_STATUS = 0x05,
        MB4_OPCODE_WRITE_INSTRUCTION = 0x07,
        MB4_OPCODE_READ_REGISTERS_0 = 0x09,
        MB4_OPCODE_WRITE_REGISTERS_0 = 0x0B,
    };

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> spiDevp;

public:
/* iC-MB4 parameters */
    const struct mb4_param MB4_SCDATA1 = { .addr = 0x07, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA2 = { .addr = 0x0F, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA3 = { .addr = 0x17, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA4 = { .addr = 0x1F, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA5 = { .addr = 0x27, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA6 = { .addr = 0x2F, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA7 = { .addr = 0x37, .pos = 7, .len = 64 };
    const struct mb4_param MB4_SCDATA8 = { .addr = 0x3F, .pos = 7, .len = 64 };

    const struct mb4_param MB4_RDATA1 = { .addr = 0x80, .pos = 7, .len = 8 };
    const struct mb4_param MB4_IDS = { .addr = 0x80, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA2 = { .addr = 0x81, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA3 = { .addr = 0x82, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA4 = { .addr = 0x83, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA5 = { .addr = 0x84, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA6 = { .addr = 0x85, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA7 = { .addr = 0x86, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA8 = { .addr = 0x87, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA9 = { .addr = 0x88, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA10 = { .addr = 0x89, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA11 = { .addr = 0x8A, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA12 = { .addr = 0x8B, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA13 = { .addr = 0x8C, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA14 = { .addr = 0x8D, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA15 = { .addr = 0x8E, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA16 = { .addr = 0x8F, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA17 = { .addr = 0x90, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA18 = { .addr = 0x91, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA19 = { .addr = 0x92, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA20 = { .addr = 0x93, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA21 = { .addr = 0x94, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA22 = { .addr = 0x95, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA23 = { .addr = 0x96, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA24 = { .addr = 0x97, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA25 = { .addr = 0x98, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA26 = { .addr = 0x99, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA27 = { .addr = 0x9A, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA28 = { .addr = 0x9B, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA29 = { .addr = 0x9C, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA30 = { .addr = 0x9D, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA31 = { .addr = 0x9E, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA32 = { .addr = 0x9F, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA33 = { .addr = 0xA0, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA34 = { .addr = 0xA1, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA35 = { .addr = 0xA2, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA36 = { .addr = 0xA3, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA37 = { .addr = 0xA4, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA38 = { .addr = 0xA5, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA39 = { .addr = 0xA6, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA40 = { .addr = 0xA7, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA41 = { .addr = 0xA8, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA42 = { .addr = 0xA9, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA43 = { .addr = 0xAA, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA44 = { .addr = 0xAB, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA45 = { .addr = 0xAC, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA46 = { .addr = 0xAD, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA47 = { .addr = 0xAE, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA48 = { .addr = 0xAF, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA49 = { .addr = 0xB0, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA50 = { .addr = 0xB1, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA51 = { .addr = 0xB2, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA52 = { .addr = 0xB3, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA53 = { .addr = 0xB4, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA54 = { .addr = 0xB5, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA55 = { .addr = 0xB6, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA56 = { .addr = 0xB7, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA57 = { .addr = 0xB8, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA58 = { .addr = 0xB9, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA59 = { .addr = 0xBA, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA60 = { .addr = 0xBB, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA61 = { .addr = 0xBC, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA62 = { .addr = 0xBD, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA63 = { .addr = 0xBE, .pos = 7, .len = 8 };
    const struct mb4_param MB4_RDATA64 = { .addr = 0xBF, .pos = 7, .len = 8 };

    const struct mb4_param MB4_SCDLEN1 = { .addr = 0xC0, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD1 = { .addr = 0xC0, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS1 = { .addr = 0xC0, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP1 = { .addr = 0xC0, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN1 = { .addr = 0xC1, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY1 = { .addr = 0xC1, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS1 = { .addr = 0xC1, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART1 = { .addr = 0xC3, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN2 = { .addr = 0xC4, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD2 = { .addr = 0xC4, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS2 = { .addr = 0xC4, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP2 = { .addr = 0xC4, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN2 = { .addr = 0xC5, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY2 = { .addr = 0xC5, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS2 = { .addr = 0xC5, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART2 = { .addr = 0xC7, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN3 = { .addr = 0xC8, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD3 = { .addr = 0xC8, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS3 = { .addr = 0xC8, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP3 = { .addr = 0xC8, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN3 = { .addr = 0xC9, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY3 = { .addr = 0xC9, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS3 = { .addr = 0xC9, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART3 = { .addr = 0xCB, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN4 = { .addr = 0xCC, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD4 = { .addr = 0xCC, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS4 = { .addr = 0xCC, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP4 = { .addr = 0xCC, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN4 = { .addr = 0xCD, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY4 = { .addr = 0xCD, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS4 = { .addr = 0xCD, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART4 = { .addr = 0xCF, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN5 = { .addr = 0xD0, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD5 = { .addr = 0xD0, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS5 = { .addr = 0xD0, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP5 = { .addr = 0xD0, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN5 = { .addr = 0xD1, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY5 = { .addr = 0xD1, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS5 = { .addr = 0xD1, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART5 = { .addr = 0xD3, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN6 = { .addr = 0xD4, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD6 = { .addr = 0xD4, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS6 = { .addr = 0xD4, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP6 = { .addr = 0xD4, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN6 = { .addr = 0xD5, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY6 = { .addr = 0xD5, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS6 = { .addr = 0xD5, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART6 = { .addr = 0xD7, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN7 = { .addr = 0xD8, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD7 = { .addr = 0xD8, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS7 = { .addr = 0xD8, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP7 = { .addr = 0xD8, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN7 = { .addr = 0xD9, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY7 = { .addr = 0xD9, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS7 = { .addr = 0xD9, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART7 = { .addr = 0xDB, .pos = 7, .len = 16 };

    const struct mb4_param MB4_SCDLEN8 = { .addr = 0xDC, .pos = 5, .len = 6 };
    const struct mb4_param MB4_ENSCD8 = { .addr = 0xDC, .pos = 6, .len = 1 };
    const struct mb4_param MB4_GRAYS8 = { .addr = 0xDC, .pos = 7, .len = 1 };
    const struct mb4_param MB4_LSTOP8 = { .addr = 0xDC, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCLEN8 = { .addr = 0xDD, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SCRCPOLY8 = { .addr = 0xDD, .pos = 6, .len = 7 };
    const struct mb4_param MB4_SELCRCS8 = { .addr = 0xDD, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SCRCSTART8 = { .addr = 0xDF, .pos = 7, .len = 16 };

    const struct mb4_param MB4_REGADR = { .addr = 0xE2, .pos = 6, .len = 7 };
    const struct mb4_param MB4_WNR = { .addr = 0xE2, .pos = 7, .len = 1 };
    const struct mb4_param MB4_REGNUM = { .addr = 0xE3, .pos = 5, .len = 6 };
    const struct mb4_param MB4_CHSEL = { .addr = 0xE4, .pos = 1, .len = 2 };
    const struct mb4_param MB4_HOLDCDM = { .addr = 0xE5, .pos = 0, .len = 1 };
    const struct mb4_param MB4_EN_MO = { .addr = 0xE5, .pos = 1, .len = 1 };
    const struct mb4_param MB4_SLAVEID = { .addr = 0xE5, .pos = 5, .len = 3 };
    const struct mb4_param MB4_IDA_TEST = { .addr = 0xE5, .pos = 3, .len = 1 };
    const struct mb4_param MB4_CMD = { .addr = 0xE5, .pos = 5, .len = 2 };
    const struct mb4_param MB4_REGVERS = { .addr = 0xE5, .pos = 6, .len = 1 };
    const struct mb4_param MB4_CTS = { .addr = 0xE5, .pos = 7, .len = 1 };

    const struct mb4_param MB4_FREQS = { .addr = 0xE6, .pos = 4, .len = 5 };
    const struct mb4_param MB4_FREQR = { .addr = 0xE6, .pos = 7, .len = 3 };
    const struct mb4_param MB4_SINGLEBANK = { .addr = 0xE7, .pos = 0, .len = 1 };
    const struct mb4_param MB4_NOCRC = { .addr = 0xE7, .pos = 1, .len = 1 };
    const struct mb4_param MB4_FREQAGS = { .addr = 0xE8, .pos = 7, .len = 8 };
    const struct mb4_param MB4_MO_BUSY = { .addr = 0xE9, .pos = 7, .len = 8 };
    const struct mb4_param MB4_REVISION = { .addr = 0xEA, .pos = 7, .len = 8 };
    const struct mb4_param MB4_VERSION = { .addr = 0xEB, .pos = 7, .len = 8 };

    const struct mb4_param MB4_SLAVELOC5 = { .addr = 0xEC, .pos = 4, .len = 1 };
    const struct mb4_param MB4_CFGCH1 = { .addr = 0xED, .pos = 1, .len = 2 };
    const struct mb4_param MB4_CFGCH2 = { .addr = 0xED, .pos = 3, .len = 2 };

    const struct mb4_param MB4_ACTnSENS = { .addr = 0xEF, .pos = 7, .len = 8 };

    const struct mb4_param MB4_EOT = { .addr = 0xF0, .pos = 0, .len = 1 };
    const struct mb4_param MB4_REGEND = { .addr = 0xF0, .pos = 2, .len = 1 };
    const struct mb4_param MB4_nREGERR = { .addr = 0xF0, .pos = 3, .len = 1 };
    const struct mb4_param MB4_nSCDERR = { .addr = 0xF0, .pos = 4, .len = 1 };
    const struct mb4_param MB4_nDELAYERR = { .addr = 0xF0, .pos = 5, .len = 1 };
    const struct mb4_param MB4_nAGSERR = { .addr = 0xF0, .pos = 6, .len = 1 };
    const struct mb4_param MB4_nERR = { .addr = 0xF0, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SVALID1 = { .addr = 0xF1, .pos = 1, .len = 1 };
    const struct mb4_param MB4_SVALID2 = { .addr = 0xF1, .pos = 3, .len = 1 };
    const struct mb4_param MB4_SVALID3 = { .addr = 0xF1, .pos = 5, .len = 1 };
    const struct mb4_param MB4_SVALID4 = { .addr = 0xF1, .pos = 7, .len = 1 };
    const struct mb4_param MB4_SVALID5 = { .addr = 0xF2, .pos = 1, .len = 1 };
    const struct mb4_param MB4_SVALID6 = { .addr = 0xF2, .pos = 3, .len = 1 };
    const struct mb4_param MB4_SVALID7 = { .addr = 0xF2, .pos = 5, .len = 1 };
    const struct mb4_param MB4_SVALID8 = { .addr = 0xF2, .pos = 7, .len = 1 };
    const struct mb4_param MB4_REGBYTES = { .addr = 0xF3, .pos = 5, .len = 6 };
    const struct mb4_param MB4_CDSSEL = { .addr = 0xF3, .pos = 6, .len = 1 };
    const struct mb4_param MB4_CDMTIMEOUT = { .addr = 0xF3, .pos = 7, .len = 1 };

    const struct mb4_param MB4_AGS = { .addr = 0xF4, .pos = 0, .len = 1 };
    const struct mb4_param MB4_INSTR = { .addr = 0xF4, .pos = 3, .len = 3 };
    const struct mb4_param MB4_INIT = { .addr = 0xF4, .pos = 4, .len = 1 };
    const struct mb4_param MB4_SWBANK = { .addr = 0xF4, .pos = 5, .len = 1 };
    const struct mb4_param MB4_HOLDBANK = { .addr = 0xF4, .pos = 6, .len = 1 };
    const struct mb4_param MB4_BREAK = { .addr = 0xF4, .pos = 7, .len = 1 };

    const struct mb4_param MB4_CLKENI = { .addr = 0xF5, .pos = 0, .len = 1 };
    const struct mb4_param MB4_ENTEST = { .addr = 0xF5, .pos = 1, .len = 1 };
    const struct mb4_param MB4_CFGIF = { .addr = 0xF5, .pos = 3, .len = 2 };
    const struct mb4_param MB4_MAFS = { .addr = 0xF5, .pos = 4, .len = 1 };
    const struct mb4_param MB4_MAVS = { .addr = 0xF5, .pos = 5, .len = 1 };
    const struct mb4_param MB4_MAFO = { .addr = 0xF5, .pos = 6, .len = 1 };
    const struct mb4_param MB4_MAVO = { .addr = 0xF5, .pos = 7, .len = 1 };

    const struct mb4_param MB4_SL1 = { .addr = 0xF8, .pos = 0, .len = 1 };
    const struct mb4_param MB4_CDS1 = { .addr = 0xF8, .pos = 1, .len = 1 };
    const struct mb4_param MB4_SL2 = { .addr = 0xF8, .pos = 2, .len = 1 };
    const struct mb4_param MB4_CDS2 = { .addr = 0xF8, .pos = 3, .len = 1 };
    const struct mb4_param MB4_SWBANKFAIL = { .addr = 0xFB, .pos = 0, .len = 1 };

private:

/* iC-MB4 defines */
#define MB4_REGISTER_SIZE	8

/* globals */
    uint8_t buf_tx[0xFF + 0x2];
    uint8_t buf_rx[0xFF + 0x2];
    uint16_t bufsize;

    void mb4_read_status(uint8_t *data_rx, uint8_t datasize);
    void mb4_write_instruction(uint8_t *data_tx, uint8_t datasize);
    void mb4_read_registers_0(uint8_t *data_rx, uint8_t datasize);
    void mb4_write_registers_0(uint8_t *data_tx, uint8_t datasize);
    uint8_t get_start_bit_number(uint8_t bit_pos, uint8_t bit_len);
    void mb4_write_registers(uint8_t start_addr, uint8_t *data_tx, uint8_t datasize);
    void mb4_read_registers(uint8_t start_addr, uint8_t *data_rx, uint8_t datasize);

public:
    MB4() {};
    bool mb4_init();
    uint64_t mb4_read_param(const struct mb4_param *param);
    void mb4_write_param(const struct mb4_param *param, uint64_t param_val);
    void mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize);
};
///* Sensor and Actuator Data */
//extern const struct mb4_param MB4_SCDATA1;
//extern const struct mb4_param MB4_SCDATA2;
//extern const struct mb4_param MB4_SCDATA3;
//extern const struct mb4_param MB4_SCDATA4;
//extern const struct mb4_param MB4_SCDATA5;
//extern const struct mb4_param MB4_SCDATA6;
//extern const struct mb4_param MB4_SCDATA7;
//extern const struct mb4_param MB4_SCDATA8;
//
///* Register Data */
//extern const struct mb4_param MB4_RDATA1;
//extern const struct mb4_param MB4_IDS;
//extern const struct mb4_param MB4_RDATA2;
//extern const struct mb4_param MB4_RDATA3;
//extern const struct mb4_param MB4_RDATA4;
//extern const struct mb4_param MB4_RDATA5;
//extern const struct mb4_param MB4_RDATA6;
//extern const struct mb4_param MB4_RDATA7;
//extern const struct mb4_param MB4_RDATA8;
//extern const struct mb4_param MB4_RDATA9;
//extern const struct mb4_param MB4_RDATA10;
//extern const struct mb4_param MB4_RDATA11;
//extern const struct mb4_param MB4_RDATA12;
//extern const struct mb4_param MB4_RDATA13;
//extern const struct mb4_param MB4_RDATA14;
//extern const struct mb4_param MB4_RDATA15;
//extern const struct mb4_param MB4_RDATA16;
//extern const struct mb4_param MB4_RDATA17;
//extern const struct mb4_param MB4_RDATA18;
//extern const struct mb4_param MB4_RDATA19;
//extern const struct mb4_param MB4_RDATA20;
//extern const struct mb4_param MB4_RDATA21;
//extern const struct mb4_param MB4_RDATA22;
//extern const struct mb4_param MB4_RDATA23;
//extern const struct mb4_param MB4_RDATA24;
//extern const struct mb4_param MB4_RDATA25;
//extern const struct mb4_param MB4_RDATA26;
//extern const struct mb4_param MB4_RDATA27;
//extern const struct mb4_param MB4_RDATA28;
//extern const struct mb4_param MB4_RDATA29;
//extern const struct mb4_param MB4_RDATA30;
//extern const struct mb4_param MB4_RDATA31;
//extern const struct mb4_param MB4_RDATA32;
//extern const struct mb4_param MB4_RDATA33;
//extern const struct mb4_param MB4_RDATA34;
//extern const struct mb4_param MB4_RDATA35;
//extern const struct mb4_param MB4_RDATA36;
//extern const struct mb4_param MB4_RDATA37;
//extern const struct mb4_param MB4_RDATA38;
//extern const struct mb4_param MB4_RDATA39;
//extern const struct mb4_param MB4_RDATA40;
//extern const struct mb4_param MB4_RDATA41;
//extern const struct mb4_param MB4_RDATA42;
//extern const struct mb4_param MB4_RDATA43;
//extern const struct mb4_param MB4_RDATA44;
//extern const struct mb4_param MB4_RDATA45;
//extern const struct mb4_param MB4_RDATA46;
//extern const struct mb4_param MB4_RDATA47;
//extern const struct mb4_param MB4_RDATA48;
//extern const struct mb4_param MB4_RDATA49;
//extern const struct mb4_param MB4_RDATA50;
//extern const struct mb4_param MB4_RDATA51;
//extern const struct mb4_param MB4_RDATA52;
//extern const struct mb4_param MB4_RDATA53;
//extern const struct mb4_param MB4_RDATA54;
//extern const struct mb4_param MB4_RDATA55;
//extern const struct mb4_param MB4_RDATA56;
//extern const struct mb4_param MB4_RDATA57;
//extern const struct mb4_param MB4_RDATA58;
//extern const struct mb4_param MB4_RDATA59;
//extern const struct mb4_param MB4_RDATA60;
//extern const struct mb4_param MB4_RDATA61;
//extern const struct mb4_param MB4_RDATA62;
//extern const struct mb4_param MB4_RDATA63;
//extern const struct mb4_param MB4_RDATA64;
//
///* Configuration Slave 1 */
//extern const struct mb4_param MB4_SCDLEN1;
//extern const struct mb4_param MB4_ENSCD1;
//extern const struct mb4_param MB4_GRAYS1;
//extern const struct mb4_param MB4_LSTOP1;
//extern const struct mb4_param MB4_SCRCLEN1;
//extern const struct mb4_param MB4_SCRCPOLY1;
//extern const struct mb4_param MB4_SELCRCS1;
//extern const struct mb4_param MB4_SCRCSTART1;
//
///* Configuration Slave 2 */
//extern const struct mb4_param MB4_SCDLEN2;
//extern const struct mb4_param MB4_ENSCD2;
//extern const struct mb4_param MB4_GRAYS2;
//extern const struct mb4_param MB4_LSTOP2;
//extern const struct mb4_param MB4_SCRCLEN2;
//extern const struct mb4_param MB4_SCRCPOLY2;
//extern const struct mb4_param MB4_SELCRCS2;
//extern const struct mb4_param MB4_SCRCSTART2;
//
///* Configuration Slave 3 */
//extern const struct mb4_param MB4_SCDLEN3;
//extern const struct mb4_param MB4_ENSCD3;
//extern const struct mb4_param MB4_GRAYS3;
//extern const struct mb4_param MB4_LSTOP3;
//extern const struct mb4_param MB4_SCRCLEN3;
//extern const struct mb4_param MB4_SCRCPOLY3;
//extern const struct mb4_param MB4_SELCRCS3;
//extern const struct mb4_param MB4_SCRCSTART3;
//
///* Configuration Slave 4 */
//extern const struct mb4_param MB4_SCDLEN4;
//extern const struct mb4_param MB4_ENSCD4;
//extern const struct mb4_param MB4_GRAYS4;
//extern const struct mb4_param MB4_LSTOP4;
//extern const struct mb4_param MB4_SCRCLEN4;
//extern const struct mb4_param MB4_SCRCPOLY4;
//extern const struct mb4_param MB4_SELCRCS4;
//extern const struct mb4_param MB4_SCRCSTART4;
//
///* Configuration Slave 5 */
//extern const struct mb4_param MB4_SCDLEN5;
//extern const struct mb4_param MB4_ENSCD5;
//extern const struct mb4_param MB4_GRAYS5;
//extern const struct mb4_param MB4_LSTOP5;
//extern const struct mb4_param MB4_SCRCLEN5;
//extern const struct mb4_param MB4_SCRCPOLY5;
//extern const struct mb4_param MB4_SELCRCS5;
//extern const struct mb4_param MB4_SCRCSTART5;
//
///* Configuration Slave 6 */
//extern const struct mb4_param MB4_SCDLEN6;
//extern const struct mb4_param MB4_ENSCD6;
//extern const struct mb4_param MB4_GRAYS6;
//extern const struct mb4_param MB4_LSTOP6;
//extern const struct mb4_param MB4_SCRCLEN6;
//extern const struct mb4_param MB4_SCRCPOLY6;
//extern const struct mb4_param MB4_SELCRCS6;
//extern const struct mb4_param MB4_SCRCSTART6;
//
///* Configuration Slave 7 */
//extern const struct mb4_param MB4_SCDLEN7;
//extern const struct mb4_param MB4_ENSCD7;
//extern const struct mb4_param MB4_GRAYS7;
//extern const struct mb4_param MB4_LSTOP7;
//extern const struct mb4_param MB4_SCRCLEN7;
//extern const struct mb4_param MB4_SCRCPOLY7;
//extern const struct mb4_param MB4_SELCRCS7;
//extern const struct mb4_param MB4_SCRCSTART7;
//
///* Configuration Slave 8 */
//extern const struct mb4_param MB4_SCDLEN8;
//extern const struct mb4_param MB4_ENSCD8;
//extern const struct mb4_param MB4_GRAYS8;
//extern const struct mb4_param MB4_LSTOP8;
//extern const struct mb4_param MB4_SCRCLEN8;
//extern const struct mb4_param MB4_SCRCPOLY8;
//extern const struct mb4_param MB4_SELCRCS8;
//extern const struct mb4_param MB4_SCRCSTART8;
//
///* Control Communication Configuration */
//extern const struct mb4_param MB4_REGADR;
//extern const struct mb4_param MB4_WNR;
//extern const struct mb4_param MB4_REGNUM;
//extern const struct mb4_param MB4_CHSEL;
//extern const struct mb4_param MB4_HOLDCDM;
//extern const struct mb4_param MB4_EN_MO;
//extern const struct mb4_param MB4_SLAVEID;
//extern const struct mb4_param MB4_IDA_TEST;
//extern const struct mb4_param MB4_CMD;
//extern const struct mb4_param MB4_REGVERS;
//extern const struct mb4_param MB4_CTS;
//
///* Master Configuration */
//extern const struct mb4_param MB4_FREQS;
//extern const struct mb4_param MB4_FREQR;
//extern const struct mb4_param MB4_SINGLEBANK;
//extern const struct mb4_param MB4_NOCRC;
//extern const struct mb4_param MB4_FREQAGS;
//extern const struct mb4_param MB4_MO_BUSY;
//extern const struct mb4_param MB4_REVISION;
//extern const struct mb4_param MB4_VERSION;
//
///* Channel Configuration */
//extern const struct mb4_param MB4_SLAVELOC5;
//extern const struct mb4_param MB4_CFGCH1;
//extern const struct mb4_param MB4_CFGCH2;
//
///* Slave Configuration 2 */
//extern const struct mb4_param MB4_ACTnSENS;
//
///* Status Information */
//extern const struct mb4_param MB4_EOT;
//extern const struct mb4_param MB4_REGEND;
//extern const struct mb4_param MB4_nREGERR;
//extern const struct mb4_param MB4_nSCDERR;
//extern const struct mb4_param MB4_nDELAYERR;
//extern const struct mb4_param MB4_nAGSERR;
//extern const struct mb4_param MB4_nERR;
//extern const struct mb4_param MB4_SVALID1;
//extern const struct mb4_param MB4_SVALID2;
//extern const struct mb4_param MB4_SVALID3;
//extern const struct mb4_param MB4_SVALID4;
//extern const struct mb4_param MB4_SVALID5;
//extern const struct mb4_param MB4_SVALID6;
//extern const struct mb4_param MB4_SVALID7;
//extern const struct mb4_param MB4_SVALID8;
//extern const struct mb4_param MB4_REGBYTES;
//extern const struct mb4_param MB4_CDSSEL;
//extern const struct mb4_param MB4_CDMTIMEOUT;
//
///* Instruction Register */
//extern const struct mb4_param MB4_AGS;
//extern const struct mb4_param MB4_INSTR;
//extern const struct mb4_param MB4_INIT;
//extern const struct mb4_param MB4_SWBANK;
//extern const struct mb4_param MB4_HOLDBANK;
//extern const struct mb4_param MB4_BREAK;
//extern const struct mb4_param MB4_CLKENI;
//extern const struct mb4_param MB4_ENTEST;
//extern const struct mb4_param MB4_CFGIF;
//extern const struct mb4_param MB4_MAFS;
//extern const struct mb4_param MB4_MAVS;
//extern const struct mb4_param MB4_MAFO;
//extern const struct mb4_param MB4_MAVO;
//
///* Status Information 2 */
//extern const struct mb4_param MB4_SL1;
//extern const struct mb4_param MB4_CDS1;
//extern const struct mb4_param MB4_SL2;
//extern const struct mb4_param MB4_CDS2;
//extern const struct mb4_param MB4_SWBANKFAIL;
///**
// * @}
// */
//
///**
// * @}
// */
//
///**
// * @defgroup MB4_Functions
// * @{
// */
///**
// * @addtogroup MB4_Functions_Basic
// * @brief Functions according to iC-MB4 SPI opcodes.
// * @{
// */
//void mb4_write_registers(uint8_t start_addr, uint8_t *data_tx, uint8_t datasize);
//void mb4_read_registers(uint8_t start_addr, uint8_t *data_rx, uint8_t datasize);
//void mb4_read_status(uint8_t *data_rx, uint8_t datasize);
//void mb4_write_instruction(uint8_t *data_tx, uint8_t datasize);
//void mb4_read_registers_0(uint8_t *data_rx, uint8_t datasize);
//void mb4_write_registers_0(uint8_t *data_tx, uint8_t datasize);
///**
// * @}
// */
//
///**
// * @addtogroup MB4_Functions_Advanced
// * @brief Functions going through a sequence of @ref MB4_Functions_Basic with enhanced data handling.
// * @{
// */
//uint64_t mb4_read_param(const struct mb4_param *param);

/**
 * @}
 */

/**
 * @}
 */

//#ifdef __cplusplus
//}
//#endif

#endif /* MB4_1SF_DRIVER_H */
