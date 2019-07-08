/*
 * treqio_memory.h
 *
 *  Created on: Nov 10, 2015
 *      Author: irw
 */

#ifndef SRC_MEMORY_TREQIO_MEMORY_H_
#define SRC_MEMORY_TREQIO_MEMORY_H_

#define TREQIO_FLASH_BLOCK_SECTOR_ERASE_SIZE		0x1000
#define TREQIO_FLASH_BLOCK_SECTOR_ERASE_MASK		(TREQIO_FLASH_BLOCK_SECTOR_ERASE_SIZE - 1)
#define TREQIO_FLASH_BLOCK_SECTOR_ERASE_ADDR_MASK	0x000FF000
#define TREQIO_FLASH_LOG_BLOCK_SIZE					0x200//0x400
#define TREQIO_FLASH_BLOCK_SIZE_INFO_SIZE			(sizeof(uint32_t))

#define TREQIO_EEPROM_ADDR							(0x50) // SDK does a left-shift of one bit and sets de least significant bit for R/W opperations
#define TREQIO_EEPROM_INFO_PAGE_ADDR				(0x51)
#define TREQIO_EEPROM_RETRIES						(50)

#define TREQIO_EEPROM_START_ADDR					((uint16_t)0x0000)

#define TREQIO_EEPROM_CONFIG_ADDR					(TREQIO_EEPROM_START_ADDR)

#define TREQIO_EEPROM_VERSION_ADDR					((uint16_t)TREQIO_EEPROM_START_ADDR)
#define TREQIO_EEPROM_VERSION_SIZE					(sizeof(uint8_t))

#define TREQIO_EEPROM_REPORT_PERIOD_ADDR			((uint16_t)(TREQIO_EEPROM_VERSION_ADDR + TREQIO_EEPROM_VERSION_SIZE))
#define TREQIO_EEPROM_REPORT_PERIOD_SIZE			(sizeof(uint32_t))

#define TREQIO_EEPROM_SAMPLING_PERIOD_ADDR			((uint16_t)(TREQIO_EEPROM_REPORT_PERIOD_ADDR + TREQIO_EEPROM_REPORT_PERIOD_SIZE))
#define TREQIO_EEPROM_SAMPLING_PERIOD_SIZE			(sizeof(uint32_t))

#define TREQIO_EEPROM_GPS_DATE_ADDR					((uint16_t)(TREQIO_EEPROM_SAMPLING_PERIOD_ADDR + TREQIO_EEPROM_SAMPLING_PERIOD_SIZE))
#define TREQIO_EEPROM_GPS_DATE_SIZE					(UBX_MGA_INI_TIME_UTC_SIZE)

#define TREQIO_EEPROM_GPS_EPHEMERIS_SIZE_ADDR		((uint16_t)(TREQIO_EEPROM_GPS_DATE_ADDR + TREQIO_EEPROM_GPS_DATE_SIZE))
#define TREQIO_EEPROM_GPS_EPHEMERIS_SIZE_SIZR		(sizeof(uint32_t))

#define SPI_FLASH_SIZE					0x1FFFFF
#define SPI_FLASH_MAX_COMMAND_SIZE		0x04
#define SPI_FLASH_ADDR_SIZE				0x03
#define SPI_FLASH_DATA_BLOCK_SIZE		0x02
#define SPI_FLASH_STATUS_REGISTER_SIZE	0x01
#define SPI_FLASH_SINGLE_COMMAND_SIZE	0x01
#define SPI_FLASH_BLOCK_ERASE_SIZE		(4096)

/* MEMORY MAP */
#define SPI_FLASH_TREQ_LOG_MASK								0x000FFFFF
#define SPI_FLASH_TREQ_BLOCK_MASK							0x000FFE00//0x000FFC00
#define SPI_FLASH_TREQ_BLOCK_PADDING_MASK					0x000001FF//0x000003FF
#define	SPI_FLASH_TREQ_DATA_START							0x000000
#define	SPI_FLASH_TREQ_DATA_SIZE							(SPI_FLASH_TREQ_MASK - SPI_FLASH_TREQ_DATA_START)
#define SPI_FLASH_TREQ_FIRMWARE								0x10000
#define SPI_FLASH_TREQ_FIRMWARE_SIZE						(512 * 1024)
#define SPI_FLASH_TREQ_EPHEMERIS							(SPI_FLASH_TREQ_FIRMWARE + SPI_FLASH_TREQ_FIRMWARE_SIZE)
#define	SPI_FLASH_TREQ_EPHEMERIS_SIZE						(256 * 1024)
#define SPI_FLASH_PAGE_SIZE									256
#define SPI_FLASH_PAGE_SIZE_MASK							(SPI_FLASH_PAGE_SIZE - 1)
#define SPI_FLASH_PAGE_MASK									0xFFFFFF00

#define SPI_FLASH_SECTOR_ERASE_PERIOD						25
//#define SPI_FLASH_BLOCK_ERASE_PERIOD						25
//#define SPI_FLASH_CHIP_ERASE_PERIOD						50
//#define SPI_FLASH_PAGE_PROGRAM_PERIOD						2

/* SPI FLASH REGISTERS */

/* CONFIGURATION */
#define SPI_FLASH_NO_OPERATION								0x00
#define SPI_FLASH_RESET_ENABLE								0x66
#define SPI_FLASH_RESET_MEMORY								0x99

#define SPI_FLASH_READ_STATUS_REGISTER						0x05
#define SPI_FLASH_STATUS_WRITE_BUSY_MASK					0b10000001

#define SPI_FLASH_WRITE_STATUS_REGISTER						0x01
#define SPI_FLASH_READ_CONFIGURATION_REGISTER				0x35

/* READ */
#define SPI_FLASH_LOW_SPEED_READ							0x03
//#define SPI_FLASH_HIGH_SPEED_READ							0x0B
#define SPI_FLASH_SET_BURST_READ							0xC0
#define SPI_FLASH_SPI_BURST_READ_WITH_WRAP					0xEC

/* IDENTIFICATION */
#define SPI_FLASH_JEDEC_ID									0x9F

/* WRITE */
#define SPI_FLASH_WRITE_ENABLE								0x06
#define SPI_FLASH_WRITE_DISABLE								0x04
#define SPI_FLASH_ERASE_4K									0x20
#define SPI_FLASH_ERASE_BLOCK								0xD8
#define SPI_FLASH_ERASE_FULL_ARRAY							0xC7
#define SPI_FLASH_PAGE_PROGRAM								0x02
#define SPI_FLASH_SUSPEND_WRITE								0xB0
#define SPI_FLASH_RESUME_WRITE								0x30

/* PROTECTION */
#define SPI_FLASH_READ_BLOCK_PROTECTION_REGISTER			0x72
#define SPI_FLASH_WRITE_BLOCK_PROTECTION_REGISTER			0x42
#define SPI_FLASH_LOCK_PROTECTION_REGISTER					0x8D
#define SPI_FLASH_NON_VOLATILE_WRITE_PROTECTION_REGISTER	0xE8
#define SPI_FLASH_GLOBAL_PROTECTION_UNLOCK					0x98
#define SPI_FLASH_READ_SECURITY_ID							0x88
#define SPI_FLASH_PROGRAM_USER_SECURITY_ID					0xA5
#define SPI_FLASH_LOCKOUT_SECURITY_ID_PROGRAMMING			0x85

/* POWER SAVING */
#define SPI_FLASH_DEEP_POWER_MODE							0xB9
#define SPI_FLASH_RELEASE_DEEP_POWER_MODE					0xAB

#define SPI_FLASH_SLOW_SPEED_READ		0x03
#define SPI_FLASH_HIGH_SPEED_READ		0x0B
#define SPI_FLASH_ERASE_4K_BLOCK		0x20
#define SPI_FLASH_ERASE_32K_BLOCK		0x52
#define SPI_FLASH_ERASE_64K_BLOCK		0xD8
#define SPI_FLASH_FULL_ERASE			0x60 //0xC7
#define SPI_FLASH_WRITE_BYTE			0x02
#define SPI_FLASH_AUTO_INCREMENT_ADDR	0xAD
#define SPI_FLASH_READ_STATUS_REGISTER	0x05
#define SPI_FLASH_ENABLE_WRITE_STATUS	0x50
#define SPI_FLASH_WRITE_STATUS_REGISTER	0x01
#define SPI_FLASH_READ_ID				0x90 //0xAB
#define SPI_FLASH_READ_JEDEC_ID			0x9F
#define SPI_FLASH_ENABLE_WRITE_REPORT	0x70
#define SPI_FLASH_DISABLE_WRITE_REPORT	0x80

#define SPI_FLASH_JEDEC_ID_SIZE			0x03

#define TREQIO_FLASH_EPHEMERIS_BLOCK_ADDR	0x00000000
#define TREQIO_FLASH_EPHEMERIS_BLOCK_SIZE	(2048000) //-> 256kBytes in size

#define TREQIO_FLASH_NRF_FW_BLOCK_ADDR		(SPI_FLASH_EPHEMERIS_BLOCK_ADDR + SPI_FLASH_EPHEMERIS_BLOCK_SIZE)
#define TREQIO_FLASH_NRF_FW_BLOCK_SIZE		(4096000) //-> 512kBytes in size

#define TREQIO_FLASH_LOGGING_BLOCK_ADDR	(SPI_FLASH_NRF_FW_BLOCK_ADDR + SPI_FLASH_NRF_FW_BLOCK_SIZE)
#define TREQIO_FLASH_LOGGING_BLOCK_SIZE	(SPI_FLASH_SIZE - SPI_FLASH_EPHEMERIS_BLOCK_SIZE - SPI_FLASH_NRF_FW_BLOCK_SIZE) //-> Rest512kBytes in size

typedef enum {
	TREQIO_I2C_READ = 0, TREQIO_I2C_WRITE,
} treq_i2c_operation_t;

typedef struct {
	treq_i2c_operation_t op;
	uint8_t addr;
	uint8_t *data;
	uint32_t size;
} treqio_i2c_command_t;

extern nrf_drv_twi_t twi;

void treqio_enable_memory(treqio_memory_status_t status);

void treqio_disable_memory(treqio_memory_status_t status);

uint32_t treqio_eeprom_read_config(treqio_system_config_t *config);

uint32_t treqio_eeprom_write_config(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_deviceId(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_deviceId(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_hwVersion(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_hwVersion(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_fwVersion(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_fwVersion(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_deviceDefaultConfig(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_deviceDefaultConfig(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_deviceReportPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_deviceReportPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_deviceLastReport(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_deviceLastReport(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_deviceLastDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_deviceLastDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_flashDataStart(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_flashDataStart(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_flashDataSize(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_flashDataSize(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_flashDataEnd(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_flashDataEnd(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_bleSamplingPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_bleSamplingPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_bleLastSample(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_bleLastSample(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_bleBeaconLifetime(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_bleBeaconLifetime(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_gpsSamplingPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_gpsSamplingPeriod(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_gpsLastSample(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_gpsLastSample(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_gpsOnlineEphemerisDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_gpsOnlineEphemerisDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_gpsOfflineEphemerisDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_gpsOfflineEphemerisDate(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_gsmDataBlockSize(treqio_system_config_t *treq);

uint32_t treqio_eeprom_write_gsmDataBlockSize(treqio_system_config_t *treq);

uint32_t treqio_eeprom_read_backlogStart(treqio_system_config_t *config);

uint32_t treqio_eeprom_write_backlogStart(treqio_system_config_t *config);

uint32_t treqio_eeprom_read_backlogEnd(treqio_system_config_t *config);

uint32_t treqio_eeprom_write_backlogEnd(treqio_system_config_t *config);

uint32_t treqio_flash_read_flashId(uint8_t *jedec);

uint32_t treqio_flash_read_status_register(uint8_t *status);

uint32_t treqio_flash_read(uint32_t addr, uint8_t *data, uint32_t size);

uint32_t treqio_flash_read_high_speed(uint32_t addr, uint8_t *data, uint32_t size);

uint32_t treqio_flash_write_enable(void);

uint32_t treqio_flash_write_disable(void);

uint32_t treqio_flash_enable_write_status_register(void);

uint32_t treqio_flash_write_status_register(uint8_t *status);

uint32_t treqio_flash_write_byte(uint32_t addr, uint8_t *data);

uint32_t treqio_flash_write_block(uint32_t addr, uint8_t *data, uint32_t size);

uint32_t treqio_flash_erase_4k_block(uint32_t addr);

uint32_t treqio_flash_erase_32k_block(uint32_t addr);

uint32_t treqio_flash_erase_64k_block(uint32_t addr);

uint32_t treqio_flash_erase_all(void);

uint32_t treqio_flash_enable_status_report(void);

uint32_t treqio_flash_disable_status_report(void);

//uint32_t treqio_flash_disable_write_protection(uint32_t addr, uint32_t size);

uint32_t treqio_flash_enable_write_protection(void);

/*
 * NEW
 */

uint8_t treqio_flash_check_write_operation(void);

uint32_t treqio_flash_enable();

uint32_t treqio_flash_disable();

uint32_t treqio_flash_write_enabled();

uint32_t treqio_flash_write(uint8_t *data, uint32_t addr, uint32_t size);

uint32_t treqio_flash_disable_write_protection();

uint32_t treqio_flash_log_data(uint8_t *data, uint32_t address, uint32_t size, uint8_t timestamp);

uint32_t treqio_flash_log_read(uint8_t *data, uint32_t address, uint32_t size);

uint32_t treqio_flash_log_extract_info_from_block(uint8_t *data, uint32_t *size);

uint32_t treqio_flash_log_read_live_log(uint8_t *data, uint32_t size, uint32_t address);

/*
 * Deletes a given flash segment. Sets the bytes in that content to 0x00;
 */
uint32_t treqio_flash_log_delete(uint32_t address, uint32_t size);

uint32_t treqio_flash_log_read_backlog_block(uint8_t *data, uint32_t *size, uint32_t address);

uint32_t treqio_flash_log_block_padding(uint32_t address, uint32_t *size);

uint32_t treqio_flash_erase(treqio_system_status_t *status);

uint32_t treqio_flash_erase_block_4k(uint32_t addr);

#endif /* SRC_MEMORY_TREQIO_MEMORY_H_ */
