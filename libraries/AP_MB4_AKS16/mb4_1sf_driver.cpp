/**
 ******************************************************************************
 * @file    mb4_1sf_driver.c
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

#include "mb4_1sf_driver.h"

/* iC-MB4 opcodes */

// SPI init
bool MB4::mb4_init()
{
    spiDevp = std::move(hal.spi->get_device("extspi"));

    if (!spiDevp) {
        hal.console->printf("mb4_init(): spiDevp is null\n");
        return false;
    }
    hal.console->printf("mb4_init(): spiDevp initialized\n");
    spiDevp->get_semaphore()->take_blocking();
    spiDevp->set_speed(AP_HAL::Device::SPEED_HIGH);
    spiDevp->get_semaphore()->give();

    return true;
}


/**
 * @brief This function is used to write data to consecutive registers in the on-chip RAM.
 *
 * @param start_addr is the address of the first register to start writing to.
 * @param data_tx is a pointer to a buffer the transmitted data is stored in.
 * @param datasize is the number of consecutive registers to be written.
 * @retval None
 */
void MB4::mb4_write_registers(uint8_t start_addr, uint8_t *data_tx, uint8_t datasize) {
	bufsize = datasize + 2;

	buf_tx[0] = MB4_OPCODE_WRITE_REGISTERS;
	buf_tx[1] = start_addr;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 2] = data_tx[i];
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to read data from consecutive registers in the on-chip RAM.
 *
 * @param start_addr is the address of the first register to start reading from.
 * @param data_rx is a pointer to a buffer the received data is written to.
 * @param datasize is the number of consecutive registers to be read.
 * @retval None
 */
void MB4::mb4_read_registers(uint8_t start_addr, uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 2;

	buf_tx[0] = MB4_OPCODE_READ_REGISTERS;
	buf_tx[1] = start_addr;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 2] = 0;
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint8_t i = 0; i < datasize; i++) {
		data_rx[i] = buf_rx[i + 2];
	}
}

/**
 * @brief This function is used to read status registers starting at address 0xF0.
 *
 * @param data_rx is a pointer to a buffer the received data is written to.
 * @param datasize is the number of status data to be read.
 * @retval None
 */
void MB4::mb4_read_status(uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 1;

	buf_tx[0] = MB4_OPCODE_READ_STATUS;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 1] = 0;
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint8_t i = 0; i < datasize; i++) {
		data_rx[i] = buf_rx[i + 1];
	}
}

/**
 * @brief This function is used to write instruction registers starting at address 0xF4.
 *
 * @param data_tx is a pointer to a buffer the transmitted data is stored in.
 * @param datasize is the number of consecutive registers to be written.
 * @retval None
 */
void MB4::mb4_write_instruction(uint8_t *data_tx, uint8_t datasize) {
	bufsize = datasize + 1;
	buf_tx[0] = MB4_OPCODE_WRITE_INSTRUCTION;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 1] = data_tx[i];
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
 * @brief This function is used to read data from consecutive registers in the on-chip RAM starting at address 0x00.
 *
 * @param data_rx is a pointer to a buffer the received data is written to.
 * @param datasize is the number of consecutive registers to be read.
 * @retval None
 */
void MB4::mb4_read_registers_0(uint8_t *data_rx, uint8_t datasize) {
	bufsize = datasize + 1;

	buf_tx[0] = MB4_OPCODE_READ_REGISTERS_0;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 1] = 0;
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);

	for (uint8_t i = 0; i < datasize; i++) {
		data_rx[i] = buf_rx[i + 1];
	}
}

/**
 * @brief This function is used to write data to consecutive registers in the on-chip RAM starting at address 0x00.
 *
 * @param data_tx is a pointer to a buffer the transmitted data is stored in.
 * @param datasize is the number of consecutive registers to be written.
 * @retval None
 */
void MB4::mb4_write_registers_0(uint8_t *data_tx, uint8_t datasize) {
	bufsize = datasize + 1;
	buf_tx[0] = MB4_OPCODE_WRITE_REGISTERS_0;

	for (uint8_t i = 0; i < datasize; i++) {
		buf_tx[i + 1] = data_tx[i];
	}

	mb4_spi_transfer(buf_tx, buf_rx, bufsize);
}

/**
  * @brief This function reads a specific chip parameter.
  *
  * @param param has to be one of the parameters defined in @ref MB4_Parameters_List.
  * @retval Value of the parameter read.
  */
uint64_t MB4::mb4_read_param(const struct mb4_param *param) {
	uint8_t datasize;

    datasize = (param->len + 7) / 8;

	uint8_t start_bit = get_start_bit_number(param->pos, param->len);
	uint8_t start_addr = param->addr - datasize + 1;
	uint8_t data_rx[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	mb4_read_registers(start_addr, data_rx, datasize);

	uint64_t param_val = 0;
	for (uint8_t i = datasize; i > 0; i--) {
		param_val <<= 8;
		param_val |= data_rx[i - 1];
	}

	param_val >>= start_bit;

	uint64_t param_mask = 0;

//	for (uint16_t i = 0; i < param->len; i++) {
//		param_mask |= (uint64_t) 1 << i;
//	}
    param_mask = (1ULL << (param->len + 1)) - 1;

	param_val &= param_mask;

	return param_val;
}

/**
  * @brief This function writes a specific chip parameter.
  *
  * @note A sequence of SPI communications is executed that will increase transmission time compared to direct register access.
  *
  * @param param has to be one of the parameters defined in @ref MB4_PARAMS.
  * @param param_val to be written to the parameter.
  * @retval None
  */
void MB4::mb4_write_param(const struct mb4_param *param, uint64_t param_val) {
	uint8_t datasize;

    datasize = (param->len + 7) / 8;

	uint64_t param_mask = 0;

//	for (uint16_t i = 0; i < param->len; i++) {
//		param_mask |= (uint64_t) 1 << i;
//	}
    param_mask = (1ULL << (param->len + 1)) - 1;

	param_val &= param_mask;

	uint8_t start_addr = param->addr - datasize + 1;
	uint8_t data_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint64_t register_value = param_val;

	if((param->pos != 7) || ((param->len % 8) != 0)) {
		mb4_read_registers(start_addr, data_buf, datasize);

		uint64_t current_val = 0;
		for (uint8_t i = datasize; i > 0; i--) {
			current_val <<= 8;
			current_val |= data_buf[i - 1];
		}

		uint8_t start_bit = get_start_bit_number(param->pos, param->len);

		param_mask <<= start_bit;
		register_value = current_val & ~param_mask;
		register_value |= param_val << start_bit;
	}

	for(uint8_t i = 0; i < datasize; i++) {
		data_buf[i] = (uint8_t)(register_value >> (i * 8));
	}

	mb4_write_registers(start_addr, data_buf, datasize);
}

/* local function definitions */
uint8_t MB4::get_start_bit_number(uint8_t bit_pos, uint8_t bit_len) {
	if (bit_len <= 8) {
		return (bit_pos - (bit_len - 1));
	}

	return 0;
}

void MB4::take_blocking()
{
    spiDevp->get_semaphore()->take_blocking();
}

void MB4::give_blocking()
{
    spiDevp->get_semaphore()->give();
}

void MB4::mb4_spi_transfer(uint8_t *data_tx, uint8_t *data_rx, uint16_t datasize)
{
//    hal.console->printf("MB4::mb4_spi_transfer(): size=%d\n", datasize);
    spiDevp->get_semaphore()->take_blocking();
//    spiDevp->set_chip_select(1);
    spiDevp->transfer_fullduplex(data_tx, data_rx, datasize);
    spiDevp->set_chip_select(0);
    spiDevp->get_semaphore()->give();

}