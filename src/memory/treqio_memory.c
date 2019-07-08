/*
 * treqio_memory.c
 *
 *  Created on: Nov 10, 2015
 *      Author: Diogo
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_error.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_error.h"
#include "spi_master.h"

#include "treqio_device_types.h"
#include "treqio_device.h"
#include "treqio_memory.h"

#ifdef DEBUG_SEGGER
#include "SEGGER_RTT.h"
#endif

extern nrf_drv_twi_t twi;

extern treqio_system_status_t treqioStatus;
extern treqio_system_config_t treqioConfig;

void treqio_enable_memory(treqio_memory_status_t status) {
	if (treqioStatus.memoryStatus == TREQIO_MEMORY_OFF) {
		nrf_gpio_pin_set (MEM_POWER_CTRL);
		treqioStatus.memoryStatus = status;
	} else {
		treqioStatus.memoryStatus |= status;
	}
	nrf_delay_ms(5);
}

void treqio_disable_memory(treqio_memory_status_t status) {
	if (treqioStatus.memoryStatus == status) {
		nrf_gpio_pin_clear (MEM_POWER_CTRL);
		treqioStatus.memoryStatus = TREQIO_MEMORY_OFF;
	} else {
		treqioStatus.memoryStatus ^= status;
	}
	nrf_delay_ms(5);
}

uint32_t treqio_eeprom_read_config(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint8_t addr[] = { (uint8_t)((TREQIO_EEPROM_CONFIG_ADDR >> 8) & 0xFF), (uint8_t)((TREQIO_EEPROM_CONFIG_ADDR) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;
	uint8_t data[sizeof(treqio_system_config_t)];

//	printf("I2C writing to address: %d\r\n", (int) (((addr[0] << 8) | addr[1]) & 0xFFFF));
	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;
	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, &(data[0]), sizeof(treqio_system_config_t), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	memcpy(config, &(data[0]), sizeof(treqio_system_config_t));

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_config(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint8_t addr[] = { (uint8_t)((TREQIO_EEPROM_CONFIG_ADDR >> 8) & 0xFF), (uint8_t)((TREQIO_EEPROM_CONFIG_ADDR) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;
	uint8_t data[sizeof(treqio_system_config_t)];

	memcpy(&(data[0]), treq, sizeof(treqio_system_config_t));

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;
	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(data[0]), sizeof(treqio_system_config_t), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_deviceId(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceId) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceId), sizeof(treq->deviceId), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_deviceId(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceId) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceId), sizeof(treq->deviceId), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_hwVersion(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->hwVersion) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->hwVersion), sizeof(treq->hwVersion), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_hwVersion(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->hwVersion) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->hwVersion), sizeof(treq->hwVersion), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_fwVersion(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->fwVersion) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->fwVersion), sizeof(treq->fwVersion), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_fwVersion(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->fwVersion) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->fwVersion), sizeof(treq->fwVersion), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_deviceDefaultConfig(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceDefaultConfig) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceDefaultConfig), sizeof(treq->deviceDefaultConfig), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_deviceDefaultConfig(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceDefaultConfig) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceDefaultConfig), sizeof(treq->deviceDefaultConfig), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_deviceReportPeriod(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (((void *) &(config->deviceReportPeriod) - (void *) config) & 0xFFFF); // + (uint16_t)(((uint8_t *) &(config->deviceReportPeriod) - (uint8_t *) config) & 0xFFFF);
	printf("Addr=%d\r\n", eepromAddr);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->deviceReportPeriod), sizeof(config->deviceReportPeriod), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_deviceReportPeriod(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (((void *) &(config->deviceReportPeriod) - (void *) config) & 0xFFFF); // + (uint16_t)(((uint8_t *) &(config->deviceReportPeriod) - (uint8_t *) config) & 0xFFFF);
	printf("Addr=%d\r\n", eepromAddr);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->deviceReportPeriod), sizeof(config->deviceReportPeriod), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

//uint32_t treqio_eeprom_read_deviceLastReport(treqio_system_config_t *config) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->deviceLastReport) - (uint8_t *) treq) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceLastReport), sizeof(treq->deviceLastReport), false) != NRF_SUCCESS)
//			&& (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}

//uint32_t treqio_eeprom_write_deviceLastReport(treqio_system_config_t *treq) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceLastReport) - (uint8_t *) treq) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceLastReport), sizeof(treq->deviceLastReport), false) != NRF_SUCCESS)
//			&& (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}

uint32_t treqio_eeprom_read_deviceLastDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceLastDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceLastDate), sizeof(treq->deviceLastDate), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_deviceLastDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->deviceLastDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->deviceLastDate), sizeof(treq->deviceLastDate), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_flashDataStart(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.start) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.start), sizeof(config->backLog.start), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_flashDataStart(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.start) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.start), sizeof(config->backLog.start), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

//uint32_t treqio_eeprom_read_flashDataSize(treqio_system_config_t *config) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->flashDataSize) - (uint8_t *) treq) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->flashDataSize), sizeof(treq->flashDataSize), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}

//uint32_t treqio_eeprom_write_flashDataSize(treqio_system_config_t *config) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->flashDataSize) - (uint8_t *) config) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->flashDataSize), sizeof(config->flashDataSize), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}

uint32_t treqio_eeprom_read_flashDataEnd(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.end) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.end), sizeof(config->backLog.end), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_flashDataEnd(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.end) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.end), sizeof(config->backLog.end), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_bleSamplingPeriod(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleSamplingPeriod) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleSamplingPeriod), sizeof(treq->bleSamplingPeriod), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_bleSamplingPeriod(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleSamplingPeriod) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleSamplingPeriod), sizeof(treq->bleSamplingPeriod), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

//uint32_t treqio_eeprom_read_bleLastSample(treqio_system_config_t *treq) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleLastSample) - (uint8_t *) treq) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleLastSample), sizeof(treq->bleLastSample), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}
//
//uint32_t treqio_eeprom_write_bleLastSample(treqio_system_config_t *treq) {
//
//	treqio_enable_memory (TREQIO_MEMORY_EEPROM);
//
//	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleLastSample) - (uint8_t *) treq) & 0xFFFF);
//	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
//	uint32_t retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	};
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_ADDR;
//	}
//
//	retries = TREQIO_EEPROM_RETRIES;
//
//	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleLastSample), sizeof(treq->bleLastSample), false) != NRF_SUCCESS) && (retries > 0)) {
//		retries--;
//	}
//
//	if (retries == 0) {
//		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//		return NRF_ERROR_INVALID_DATA;
//	}
//
//	treqio_disable_memory(TREQIO_MEMORY_EEPROM);
//
//	return NRF_SUCCESS;
//}

uint32_t treqio_eeprom_read_bleBeaconLifetime(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleBeaconLifetime) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleBeaconLifetime), sizeof(treq->bleBeaconLifetime), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_bleBeaconLifetime(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->bleBeaconLifetime) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->bleBeaconLifetime), sizeof(treq->bleBeaconLifetime), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_gpsSamplingPeriod(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->gpsSamplingPeriod) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->gpsSamplingPeriod), sizeof(config->gpsSamplingPeriod), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_gpsSamplingPeriod(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->gpsSamplingPeriod) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->gpsSamplingPeriod), sizeof(uint32_t), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_gpsLastSample(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->gpsLastKnownLocation) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->gpsLastKnownLocation), sizeof(config->gpsLastKnownLocation), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		nrf_gpio_pin_clear (LED_RED);
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_gpsLastSample(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->gpsLastKnownLocation) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->gpsLastKnownLocation), sizeof(config->gpsLastKnownLocation), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_gpsOnlineEphemerisDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gpsOnlineEphemerisDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gpsOnlineEphemerisDate), sizeof(treq->gpsOnlineEphemerisDate), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_gpsOnlineEphemerisDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gpsOnlineEphemerisDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gpsOnlineEphemerisDate), sizeof(treq->gpsOnlineEphemerisDate), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_gpsOfflineEphemerisDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gpsOfflineEphemerisDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gpsOfflineEphemerisDate), sizeof(treq->gpsOfflineEphemerisDate), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_gpsOfflineEphemerisDate(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gpsOfflineEphemerisDate) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gpsOfflineEphemerisDate), sizeof(treq->gpsOfflineEphemerisDate), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_gsmDataBlockSize(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gsmDataBlockSize) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gsmDataBlockSize), sizeof(treq->gsmDataBlockSize), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_gsmDataBlockSize(treqio_system_config_t *treq) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(treq->gsmDataBlockSize) - (uint8_t *) treq) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(treq->gsmDataBlockSize), sizeof(treq->gsmDataBlockSize), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

/*
 * Backlog eeprom write/read functions for individual variables.
 */

uint32_t treqio_eeprom_read_backlogStart(treqio_system_config_t *config) {

	config->backLog.start = 0;

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.start) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.start), sizeof(config->backLog.start), false) != NRF_SUCCESS)
			&& (retries > 0)) {
		retries--;
	}

	config->backLog.start &= SPI_FLASH_TREQ_LOG_MASK;

	if (config->backLog.start == SPI_FLASH_TREQ_LOG_MASK) {
		config->backLog.start = 0;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_backlogStart(treqio_system_config_t *config) {

	treqio_system_config_t tmp;
	treqio_eeprom_read_backlogStart(&tmp);

	config->backLog.start = config->backLog.start & SPI_FLASH_TREQ_LOG_MASK;

	if (tmp.backLog.start != config->backLog.start) {

		treqio_enable_memory (TREQIO_MEMORY_EEPROM);

		uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.start) - (uint8_t *) config) & 0xFFFF);
		uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
		uint32_t retries = TREQIO_EEPROM_RETRIES;

		while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
			retries--;
		};

		if (retries == 0) {
			treqio_disable_memory(TREQIO_MEMORY_EEPROM);
			return NRF_ERROR_INVALID_ADDR;
		}

		retries = TREQIO_EEPROM_RETRIES;

		while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.start), sizeof(uint32_t), false) != NRF_SUCCESS) && (retries > 0)) {
			retries--;
		}

		if (retries == 0) {
			treqio_disable_memory(TREQIO_MEMORY_EEPROM);
			return NRF_ERROR_INVALID_DATA;
		}

		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
	}
	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_read_backlogEnd(treqio_system_config_t *config) {

	treqio_enable_memory (TREQIO_MEMORY_EEPROM);

	config->backLog.end = 0;

	uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.end) - (uint8_t *) config) & 0xFFFF);
	uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
	uint32_t retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	};

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_ADDR;
	}

	retries = TREQIO_EEPROM_RETRIES;

	while ((nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.end), sizeof(config->backLog.end), false) != NRF_SUCCESS) && (retries > 0)) {
		retries--;
	}

	config->backLog.end &= SPI_FLASH_TREQ_LOG_MASK;
	if (config->backLog.end == SPI_FLASH_TREQ_LOG_MASK) {
		config->backLog.end = 0;
	}

	if (retries == 0) {
		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
		return NRF_ERROR_INVALID_DATA;
	}

	treqio_disable_memory(TREQIO_MEMORY_EEPROM);

	return NRF_SUCCESS;
}

uint32_t treqio_eeprom_write_backlogEnd(treqio_system_config_t *config) {

	treqio_system_config_t tmp;
	treqio_eeprom_read_backlogEnd(&tmp);

	config->backLog.end = config->backLog.end & SPI_FLASH_TREQ_LOG_MASK;

	if (tmp.backLog.end != config->backLog.end) {

		treqio_enable_memory (TREQIO_MEMORY_EEPROM);

		uint16_t eepromAddr = TREQIO_EEPROM_CONFIG_ADDR + (uint16_t)(((uint8_t *) &(config->backLog.end) - (uint8_t *) config) & 0xFFFF);
		uint8_t addr[] = { (uint8_t)((eepromAddr >> 8) & 0xFF), (uint8_t)((eepromAddr) & 0xFF) };
		uint32_t retries = TREQIO_EEPROM_RETRIES;

		while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, &(addr[0]), sizeof(addr), true) != NRF_SUCCESS) && (retries > 0)) {
			retries--;
		};

		if (retries == 0) {
			treqio_disable_memory(TREQIO_MEMORY_EEPROM);
			return NRF_ERROR_INVALID_ADDR;
		}

		retries = TREQIO_EEPROM_RETRIES;

		while ((nrf_drv_twi_tx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &(config->backLog.end), sizeof(uint32_t), false) != NRF_SUCCESS) && (retries > 0)) {
			retries--;
		}

		if (retries == 0) {
			treqio_disable_memory(TREQIO_MEMORY_EEPROM);
			return NRF_ERROR_INVALID_DATA;
		}

		treqio_disable_memory(TREQIO_MEMORY_EEPROM);
	}

	return NRF_SUCCESS;
}

/*
 * NEW
 */

/*
 *
 */
uint8_t treqio_flash_check_write_operation(void) {
	uint32_t err_code;

	uint8_t tx[] = { SPI_FLASH_READ_STATUS_REGISTER };
	uint8_t rx[2];

	do {
		err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), 1, &(rx[0]), 2);
		APP_ERROR_CHECK(err_code);
	} while ((rx[1] & SPI_FLASH_STATUS_WRITE_BUSY_MASK) != 0);

	return false;
}

uint32_t treqio_flash_enable() {
//	nrf_gpio_pin_set (MEM_POWER_CTRL);
	treqioStatus.memoryStatus |= TREQIO_MEMORY_FLASH;

//	nrf_delay_ms(10);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_disable() {
//	nrf_gpio_pin_clear (MEM_POWER_CTRL);
	treqioStatus.memoryStatus ^= TREQIO_MEMORY_FLASH;
//	nrf_delay_ms(10);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_write_enabled() {
	uint32_t err_code;

	uint8_t tx[] = { SPI_FLASH_WRITE_ENABLE };

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), sizeof(tx), NULL, 0);
	APP_ERROR_CHECK(err_code);

//	nrf_delay_ms (SPI_FLASH_PAGE_PROGRAM_PERIOD);
	treqio_flash_check_write_operation();

	return NRF_SUCCESS;
}

uint32_t treqio_flash_write(uint8_t *data, uint32_t addr, uint32_t size) {
	uint32_t err_code;

	uint8_t tx[SPI_FLASH_MAX_COMMAND_SIZE + size];
	memset(&(tx[0]), 0x00, sizeof(tx));

	tx[0] = SPI_FLASH_PAGE_PROGRAM;
	tx[1] = (uint8_t)((addr >> 16) & 0xFF);
	tx[2] = (uint8_t)((addr >> 8) & 0xFF);
	tx[3] = (uint8_t)(addr & 0xFF);
	memcpy(&(tx[SPI_FLASH_MAX_COMMAND_SIZE]), &(data[0]), size);

	err_code = treqio_flash_write_enabled();
	APP_ERROR_CHECK(err_code);

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), (SPI_FLASH_MAX_COMMAND_SIZE + size), NULL, 0);
	APP_ERROR_CHECK(err_code);

	treqio_flash_check_write_operation();

	return NRF_SUCCESS;
}

uint32_t treqio_flash_disable_write_protection() {
	uint32_t err_code;

	err_code = treqio_flash_write_enabled();
	APP_ERROR_CHECK(err_code);

	uint8_t tx[] = { SPI_FLASH_WRITE_BLOCK_PROTECTION_REGISTER, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), 7, NULL, 0);
	APP_ERROR_CHECK(err_code);

	treqio_flash_check_write_operation();

	return NRF_SUCCESS;
}

uint32_t treqio_flash_write_data(uint8_t *data, uint32_t addr, uint32_t size) {
	uint32_t err_code;

	err_code = treqio_flash_enable();
	APP_ERROR_CHECK(err_code);

	err_code = treqio_flash_disable_write_protection();
	APP_ERROR_CHECK(err_code);

	uint32_t data_idx = 0;

#ifdef DEBUG_DEEP_MEMORY2
	SEGGER_RTT_printf(0, "\r\nAddr: %.8X - Size: %d -> ", addr, size);
	SEGGER_RTT_printf(0, "Data to log:\r\n");
	for (uint32_t idx = 0; idx < size; idx++) {
		SEGGER_RTT_printf(0, "%.2X", data[idx]);
		nrf_delay_us(50);
	}
	SEGGER_RTT_printf(0, "\r\n");
#endif

	while (size > 0) {
//		if (((addr + data_idx) / SPI_FLASH_PAGE_SIZE) != (((addr + data_idx) + size) / SPI_FLASH_PAGE_SIZE)) {
		if (((addr + data_idx) & SPI_FLASH_PAGE_MASK) != ((addr + data_idx + size) & SPI_FLASH_PAGE_MASK)) {
			uint32_t remainder = (((addr + data_idx) ^ SPI_FLASH_PAGE_SIZE_MASK) & SPI_FLASH_PAGE_SIZE_MASK) + 1;
//			uint32_t remainder = (((addr + data_idx) ^ SPI_FLASH_PAGE_SIZE_MASK) & SPI_FLASH_PAGE_SIZE_MASK) + 1;
//			uint32_t remainder = (((addr + data_idx) ^ SPI_FLASH_PAGE_SIZE_MASK)) + 1;

#ifdef DEBUG_DEEP_MEMORY2
			SEGGER_RTT_printf(0, "Not enough space on this page...\r\nAddr: %.8X\r\nSize: %d\r\nRemainder: %d\r\n", addr, size, remainder);
			SEGGER_RTT_printf(0, "\r\nAddr: %.8X - Size: %d -> ", addr + data_idx, size - remainder);
			for (uint32_t idx = 0; idx < remainder; idx++) {
				SEGGER_RTT_printf(0, "%.2X", data[data_idx + idx]);
				nrf_delay_us(50);
			}
#endif

			err_code = treqio_flash_write(&(data[data_idx]), ((addr + data_idx) & SPI_FLASH_TREQ_LOG_MASK), remainder);
			APP_ERROR_CHECK(err_code);

#ifdef DEBUG_DEEP_MEMORY2
			uint8_t dbg[remainder];
			memset(&(dbg[0]), 0x00, remainder);

			treqio_flash_log_read(&(dbg[0]), ((addr + data_idx) & SPI_FLASH_TREQ_LOG_MASK), remainder);

			for (uint32_t idx = 0; idx < remainder; idx++) {
				SEGGER_RTT_printf(0, "%.2X", dbg[idx]);
				nrf_delay_us(50);
			}
			SEGGER_RTT_printf(0, "\r\n");
#endif

			size -= remainder;
			data_idx += remainder;
		} else {

#ifdef DEBUG_DEEP_MEMORY2
			SEGGER_RTT_printf(0, "Enough space on this page\r\n");
			SEGGER_RTT_printf(0, "\r\nAddr: %.8X - Size: %d -> ", addr + data_idx, size);
			for (uint32_t idx = 0; idx < size; idx++) {
				SEGGER_RTT_printf(0, "%.2X", data[data_idx + idx]);
				nrf_delay_us(50);
			}
#endif

			err_code = treqio_flash_write(&(data[data_idx]), ((addr + data_idx) & SPI_FLASH_TREQ_LOG_MASK), size);
			APP_ERROR_CHECK(err_code);

#ifdef DEBUG_DEEP_MEMORY2
			uint8_t dbg[size];
			memset(&(dbg[0]), 0x00, size);

			treqio_flash_log_read(&(dbg[0]), ((addr + data_idx) & SPI_FLASH_TREQ_LOG_MASK), size);

			for (uint32_t idx = 0; idx < size; idx++) {
				SEGGER_RTT_printf(0, "%.2X", dbg[idx]);
				nrf_delay_us(50);
			}
			SEGGER_RTT_printf(0, "\r\n");
#endif
			size -= size;
			data_idx += size;
//			} else {
//				err_code = treqio_flash_write(data, addr, size);
//				APP_ERROR_CHECK(err_code);
//				size -= SPI_FLASH_PAGE_SIZE;
//				addr += SPI_FLASH_PAGE_SIZE;
//			}
		}
	}

//	for (uint32_t idx = 0; idx < size;) {
//		if ((addr / SPI_FLASH_PAGE_SIZE) != ((addr + size) / SPI_FLASH_PAGE_SIZE)) {
//			uint32_t remainder = SPI_FLASH_PAGE_SIZE - (addr % SPI_FLASH_PAGE_SIZE);
//
//		}
//	}

//	if ((addr / SPI_FLASH_PAGE_SIZE) != ((addr + size) / SPI_FLASH_PAGE_SIZE)) {
//
//		uint32_t remainder = SPI_FLASH_PAGE_SIZE - (addr % SPI_FLASH_PAGE_SIZE);
//
//		err_code = treqio_flash_write(data, addr, remainder);
//		APP_ERROR_CHECK(err_code);
//
//		addr += remainder;
//		err_code = treqio_flash_write(&(data[remainder]), addr, size - remainder);
//		APP_ERROR_CHECK(err_code);
//
//		addr += (size - remainder);
//	} else {
//		err_code = treqio_flash_write(data, addr, size);
//		APP_ERROR_CHECK(err_code);
//
//	}

	err_code = treqio_flash_disable();
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_data(uint8_t *data, uint32_t address, uint32_t size, uint8_t timestamp) {

//	treqio_debug_live_log();

	uint32_t err_code;

	uint32_t addr = (SPI_FLASH_TREQ_DATA_START + address) & SPI_FLASH_TREQ_LOG_MASK;

	uint8_t frame[TREQIO_FRAME_SIZE];
	memset(&(frame[0]), 0x00, TREQIO_FRAME_SIZE);
	uint32_t frameIdx = 0;
	uint32_t frameSize = 0;

#ifdef DEBUG_DEEP_MEMORY2
	SEGGER_RTT_printf(0, "Writing on: %d -> %.8X\r\n", address, addr);
	SEGGER_RTT_printf(0, "Data to log:\r\n");
	for (uint32_t idx = 0; idx < size; idx++) {
		SEGGER_RTT_printf(0, "%.2X", data[idx]);
		nrf_delay_us(50);
	}
	SEGGER_RTT_printf(0, "\r\n");
#endif

	if (timestamp == true) {
		treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_TIMESTAMP_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
		frameIdx += frameSize;

		treqioStatus.delta = 0;

	} else {
		if ((treqioStatus.liveLog.size + size + TREQIO_INFO_DELTATIME_TAG_SIZE) > TREQIO_FLASH_LOG_BLOCK_SIZE) {
			treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_TIMESTAMP_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
			frameIdx += frameSize;

			treqioStatus.delta = 0;

		}
	}

	/*
	 * Clear the timestamp requirement.
	 */
	treqioStatus.newTimestamp = false;

	treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_DELTATIME_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_gps_position_t));
//	SEGGER_RTT_printf(0, "Delta: %d - %.2X %.2X %.2X %.2X\r\n", treqioStatus.delta, frame[frameIdx + 2], frame[frameIdx + 3], frame[frameIdx + 4],
//			frame[frameIdx + 5]);
	frameIdx += frameSize;

#ifdef DEBUG_DEEP_MEMORY
	SEGGER_RTT_printf(0, "\r\nLive Log Size: %d\r\nSize: %d\r\nFrameIdx: %d\r\n", treqioStatus.liveLog.size, size, frameIdx);
#endif

	if ((treqioStatus.liveLog.size + size + frameIdx) <= TREQIO_FLASH_LOG_BLOCK_SIZE) {

		if ((addr & TREQIO_FLASH_BLOCK_SECTOR_ERASE_MASK) == 0) {
//			SEGGER_RTT_printf(0, "Delete block: %.8X\r\n", addr);
			err_code = treqio_flash_erase_block_4k(addr);
			APP_ERROR_CHECK(err_code);
		}

		treqio_flash_write_data(&(frame[0]), addr, frameIdx);
		addr = (addr + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.size = (treqioStatus.liveLog.size + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.end = (treqioStatus.liveLog.end + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;

		treqio_flash_write_data(&(data[0]), addr, size);
		treqioStatus.liveLog.size = (treqioStatus.liveLog.size + size) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.end = (treqioStatus.liveLog.end + size) & SPI_FLASH_TREQ_LOG_MASK;

		if ((treqioStatus.liveLog.size & SPI_FLASH_PAGE_SIZE) == TREQIO_FLASH_LOG_BLOCK_SIZE) {

			treqioStatus.liveLog.size = 0;
			treqioStatus.liveLog.start = (treqioStatus.liveLog.end) & SPI_FLASH_TREQ_LOG_MASK;
			treqioConfig.backLog.end = (treqioStatus.liveLog.start) & SPI_FLASH_TREQ_LOG_MASK;
			treqio_eeprom_write_backlogEnd(&treqioConfig);
//			treqioConfig.backLog.end = ((address / TREQIO_FLASH_BLOCK_SIZE) + 1) * TREQIO_FLASH_BLOCK_SIZE;

//			treqioStatus.nextBlockCounter++;
//			if (treqioStatus.nextBlockCounter >= (uint8_t)((SPI_FLASH_BLOCK_ERASE_SIZE / TREQIO_FLASH_BLOCK_SIZE) - 1)) {
//				err_code = treqio_flash_erase_block_4k(((SPI_FLASH_TREQ_DATA_START + address) & SPI_FLASH_TREQ_MASK) + 0x1000);
//				APP_ERROR_CHECK(err_code);
//				treqioStatus.nextBlockCounter = 0;
//			}
			treqioStatus.newTimestamp = true;

		}
	} else {
		uint32_t jump = 0;
		treqio_flash_log_block_padding(address, &(jump));

#ifdef DEBUG_BLOCK_PADDING_SIZE
		SEGGER_RTT_printf(0, "\r\nPadding Size %d\r\n", jump);
#endif
		addr = (addr + jump) & SPI_FLASH_TREQ_LOG_MASK;

		if ((addr & TREQIO_FLASH_BLOCK_SECTOR_ERASE_MASK) == 0) {
//			SEGGER_RTT_printf(0, "Delete block: %.8X\r\n", addr);
			err_code = treqio_flash_erase_block_4k(addr);
			APP_ERROR_CHECK(err_code);
		}

		treqioStatus.liveLog.size = 0;
		treqioStatus.liveLog.end = (treqioStatus.liveLog.end + jump) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.start = (treqioStatus.liveLog.end) & SPI_FLASH_TREQ_LOG_MASK;
		treqioConfig.backLog.end = (treqioStatus.liveLog.start) & SPI_FLASH_TREQ_LOG_MASK;
		treqio_eeprom_write_backlogEnd(&treqioConfig);

//		treqioConfig.logEndIdx = addr;
//		addr = (addr + SPI_FLASH_TREQ_DATA) & SPI_FLASH_TREQ_MASK;
//		treqioStatus.logCurrentBlockSize = 0;

		treqio_flash_write_data(&(frame[0]), addr, frameIdx);
		addr = (addr + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.size = (treqioStatus.liveLog.size + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.end = (treqioStatus.liveLog.end + frameIdx) & SPI_FLASH_TREQ_LOG_MASK;

		treqio_flash_write_data(&(data[0]), addr, size);
		treqioStatus.liveLog.size = (treqioStatus.liveLog.size + size) & SPI_FLASH_TREQ_LOG_MASK;
		treqioStatus.liveLog.end = (treqioStatus.liveLog.end + size) & SPI_FLASH_TREQ_LOG_MASK;

	}

//#ifdef DEBUG_MEMORY
//	{
//		uint8_t dbg[1024];
//
//		for (uint32_t idx = 0; idx < TREQIO_FLASH_BLOCK_SIZE; idx++) {
//			dbg[idx] = ' ';
//		}
//
//		uint32_t tmp_addr = ((address / TREQIO_FLASH_BLOCK_SIZE))*TREQIO_FLASH_BLOCK_SIZE;
//		treqio_flash_log_read(&(dbg[0]), tmp_addr, TREQIO_FLASH_BLOCK_SIZE);
//
//		SEGGER_RTT_printf(0, "\r\n\r\nAddr: %d\r\n", tmp_addr);
//
//		for (uint32_t idx = 0; idx < TREQIO_FLASH_BLOCK_SIZE; idx++) {
//			SEGGER_RTT_printf(0, "%.2X", dbg[idx]);
//		}
//
//		SEGGER_RTT_printf(0, "\r\n\r\n");
//	}
//#endif
//
//#ifdef DEBUG_MEMORY
//	{
//		uint8_t dbg[1024];
//
//		for (uint32_t idx = 0; idx < TREQIO_FLASH_BLOCK_SIZE; idx++) {
//			dbg[idx] = ' ';
//		}
//
//		uint32_t tmp_addr = ((address / TREQIO_FLASH_BLOCK_SIZE))*TREQIO_FLASH_BLOCK_SIZE;
//		treqio_flash_log_read(&(dbg[0]), tmp_addr, TREQIO_FLASH_BLOCK_SIZE);
//
//		uint32_t data_size = 0;
//		treqio_flash_log_extract_info_from_block(&(dbg[0]), &(data_size));
//
//		SEGGER_RTT_printf(0, "\r\n\r\nAddr: %d\r\n", tmp_addr);
//
//		for (uint32_t idx = 0; idx < data_size; idx++) {
//			SEGGER_RTT_printf(0, "%.2X", dbg[idx]);
//		}
//
//		SEGGER_RTT_printf(0, "\r\n\r\n");
//	}
//#endif

	err_code = treqio_flash_disable();
	APP_ERROR_CHECK(err_code);

//	SEGGER_RTT_printf(0, "Write complete\r\n");

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_read(uint8_t *data, uint32_t address, uint32_t size) {
	uint32_t err_code;

	uint32_t addr = (SPI_FLASH_TREQ_DATA_START + address) & SPI_FLASH_TREQ_LOG_MASK;

#ifdef DEBUG_DEEP_MEMORY2
	SEGGER_RTT_printf(0, "\r\nAddr: %.8X - Size: %d -> \r\n", addr, size);
#endif

#ifdef DEBUG_MEMORY
//	SEGGER_RTT_printf(0, "\r\nAddress to read from: %.8X\r\nReal Address to read from: %.8X\r\n", address, addr);
#endif

//	err_code = treqio_flash_enable(status);
//	APP_ERROR_CHECK(err_code);

	uint8_t tx[] = { SPI_FLASH_LOW_SPEED_READ, (uint8_t)((addr >> 16) & 0xFF), (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF) };
	uint8_t rx[TREQIO_FLASH_LOG_BLOCK_SIZE + sizeof(tx)];

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), sizeof(tx), &(rx[0]), size + sizeof(tx));
	APP_ERROR_CHECK(err_code);

	memcpy(&(data[0]), &(rx[sizeof(tx)]), size);

//	err_code = treqio_flash_disable(status);
//	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_extract_info_from_block(uint8_t *data, uint32_t *size) {
	uint8_t clean_block[*size];
	uint32_t clean_block_idx = 0;

	uint32_t tag_size = 0;

#ifdef DEBUG_INFO_EXTRACTION
	SEGGER_RTT_printf(0, "Received Size: %d\r\nTmp size: %d\r\n", *size, sizeof(clean_block));
#endif

	memset(&(clean_block[0]), 0x00, *size);

	for (uint32_t idx = 0; idx < (*size); idx++) {
		if ((data[idx] != 0x00) && (data[idx] != 0xFF)) {
			tag_size = (uint32_t) data[idx + 1];

			memcpy(&(clean_block[clean_block_idx]), &(data[idx]), tag_size + TREQIO_WEB_TAG_HEADER_SIZE);

#ifdef DEBUG_INFO_EXTRACTION
			for (uint32_t i = 0; i < tag_size + 2; i++) {
				SEGGER_RTT_printf(0, "%.2X ", data[idx + i]);
				nrf_delay_us(100);
			}
			SEGGER_RTT_printf(0, "\r\n");
#endif

			clean_block_idx += tag_size + TREQIO_WEB_TAG_HEADER_SIZE;
			idx += tag_size + TREQIO_WEB_TAG_HEADER_SIZE - 1;
		}
	}

	memcpy(size, &(clean_block_idx), sizeof(uint32_t));
	memcpy(&(data[0]), &(clean_block[0]), clean_block_idx);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_read_live_log(uint8_t *data, uint32_t address, uint32_t size) {

	uint32_t err_code;

	uint8_t tmp[TREQIO_FLASH_LOG_BLOCK_SIZE];
	memset(&(tmp[0]), 0x00, TREQIO_FLASH_LOG_BLOCK_SIZE);

//	err_code = treqio_flash_log_read(&(tmp[0]), address, size);
	err_code = treqio_flash_log_read(&(data[0]), address, size);
	APP_ERROR_CHECK(err_code);

	/*
	 * ToDo: Process this block here
	 */

//	uint32_t addr = ((address / TREQIO_FLASH_BLOCK_SIZE)) * TREQIO_FLASH_BLOCK_SIZE;
//	if (SPI_FLASH_TREQ_DATA + address + size > SPI_FLASH_TREQ_MASK) {
//		/*
//		 * Needs to be read in two steps; One up to SPI_FLASH_TREQ_MASK; and another to read the remainder from SPI_FLASH_TREQ_DATA
//		 */
//
//		err_code = treqio_flash_log_read(&(tmp[0]), address, size);
//		APP_ERROR_CHECK(err_code);
//	} else {
//		/*
//		 * Can be read in just on step
//		 */
//		err_code = treqio_flash_log_read(&(tmp[0]), address, TREQIO_FLASH_BLOCK_SIZE);
//		APP_ERROR_CHECK(err_code);
//	}
	/*
	 * Process and retreive the valid data from the read data
	 */
//	treqio_flash_log_extract_info_from_block(&(tmp[0]), data_length);
//	memcpy(&(data[0]), &(tmp[0]), *data_length);
	return NRF_SUCCESS;
}

/*
 * Deletes a given flash segment. Sets the bytes in that content to 0x00;
 */
uint32_t treqio_flash_log_delete(uint32_t address, uint32_t size) {
	uint8_t padding_block[size];
	memset(&(padding_block[0]), 0x00, size);

	uint32_t addr = (SPI_FLASH_TREQ_DATA_START + address) & SPI_FLASH_TREQ_LOG_MASK;

	treqio_flash_write_data(&(padding_block[0]), addr, size);

//	nrf_delay_ms (SPI_FLASH_CHIP_ERASE_PERIOD);

	treqio_flash_check_write_operation();

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_read_backlog_block(uint8_t *data, uint32_t *size, uint32_t address) {

	uint32_t err_code;

	uint8_t tmp[TREQIO_FLASH_LOG_BLOCK_SIZE];

//	uint32_t addr = ((address / TREQIO_FLASH_BLOCK_SIZE)) * TREQIO_FLASH_BLOCK_SIZE;
	uint32_t addr = address & SPI_FLASH_TREQ_BLOCK_MASK;

	err_code = treqio_flash_log_read(&(tmp[0]), addr, TREQIO_FLASH_LOG_BLOCK_SIZE);
	APP_ERROR_CHECK(err_code);

	*size = TREQIO_FLASH_LOG_BLOCK_SIZE;

	treqio_flash_log_extract_info_from_block(&(tmp[0]), size);

	memcpy(&(data[0]), &(tmp[0]), *size);

#ifdef DEBUG_DEEP_MEMORY
	SEGGER_RTT_printf(0, "\r\n\r\n");
	for (uint32_t idx = 0; idx < *size; idx++) {
		SEGGER_RTT_printf(0, "%.2X ", data[idx]);
		nrf_delay_us(100);
	}
	SEGGER_RTT_printf(0, "\r\n\r\n");
#endif

	return NRF_SUCCESS;
}

uint32_t treqio_flash_log_block_padding(uint32_t address, uint32_t *size) {

	/*
	 * ToDo: get data from start if the log jumps
	 */

//	uint32_t padding_size = (((address / TREQIO_FLASH_BLOCK_SIZE) + 1) * TREQIO_FLASH_BLOCK_SIZE) - address;
//	uint32_t padding_size = (TREQIO_FLASH_LOG_BLOCK_SIZE - (address & SPI_FLASH_TREQ_BLOCK_MASK)) & SPI_FLASH_TREQ_BLOCK_PADDING_MASK;
	uint32_t padding_size = ((address & SPI_FLASH_TREQ_BLOCK_PADDING_MASK) ^ SPI_FLASH_TREQ_BLOCK_PADDING_MASK) + 1;
	uint32_t addr = (SPI_FLASH_TREQ_DATA_START + address) & SPI_FLASH_TREQ_LOG_MASK;

#ifdef DEBUG_DEEP_MEMORY
	SEGGER_RTT_printf(0, "Addr: %d\r\nPadding addr: 0x%.8X\r\nPadding size: %d\r\n", address, addr, padding_size);
#endif

	uint8_t padding[padding_size];
	memset(&(padding[0]), 0x00, padding_size);

	treqio_flash_write_data(&(padding[0]), addr, padding_size);

	memcpy(size, &(padding_size), sizeof(uint32_t));

	return NRF_SUCCESS;
}

uint32_t treqio_flash_erase(treqio_system_status_t * status) {

	uint32_t err_code;

	err_code = treqio_flash_enable(status);
	APP_ERROR_CHECK(err_code);

	err_code = treqio_flash_disable_write_protection();
	APP_ERROR_CHECK(err_code);

	err_code = treqio_flash_write_enable();
	APP_ERROR_CHECK(err_code);

	uint8_t tx[] = { SPI_FLASH_ERASE_FULL_ARRAY };

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), sizeof(tx), NULL, 0);
	APP_ERROR_CHECK(err_code);

//	nrf_delay_ms (SPI_FLASH_CHIP_ERASE_PERIOD);
	treqio_flash_check_write_operation();

	err_code = treqio_flash_disable(status);
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_erase_block_4k(uint32_t addr) {
	uint32_t err_code;

	err_code = treqio_flash_disable_write_protection(0, SPI_FLASH_SIZE);
	APP_ERROR_CHECK(err_code);

	err_code = treqio_flash_write_enable();
	APP_ERROR_CHECK(err_code);

	addr &= SPI_FLASH_SIZE;

	uint8_t tx[] = { SPI_FLASH_ERASE_4K, ((addr >> 16) & 0xFF), ((addr >> 8) & 0xFF), (addr & 0xFF) };

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), SPI_FLASH_MAX_COMMAND_SIZE, NULL, 0);
	APP_ERROR_CHECK(err_code);

//	nrf_delay_ms (SPI_FLASH_SECTOR_ERASE_PERIOD);
	treqio_flash_check_write_operation();

	return NRF_SUCCESS;
}

uint32_t treqio_flash_write_enable(void) {

	uint32_t err_code;
	uint8_t spiTx[] = { SPI_FLASH_WRITE_ENABLE };

	err_code = spi_master_send_recv(SPI_MASTER, &(spiTx[0]), sizeof(spiTx), NULL, 0);
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

uint32_t treqio_flash_read_jedec(treqio_system_status_t * status, uint8_t * jedec) {

	uint32_t err_code;

	uint8_t tx[] = { SPI_FLASH_JEDEC_ID };
	uint8_t rx[4];

	err_code = treqio_flash_enable(status);
	APP_ERROR_CHECK(err_code);

	err_code = spi_master_send_recv(SPI_MASTER, &(tx[0]), sizeof(tx), &(rx[0]), sizeof(rx));
	APP_ERROR_CHECK(err_code);

	memcpy(&(jedec[0]), &(rx[sizeof(tx)]), 3);

	return NRF_SUCCESS;
}

