/*
 * device.c
 *
 *  Created on: Dec 1, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_error.h"
#include "app_uart.h"
#include "app_timer_appsh.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_adc.h"
#include "nrf_delay.h"
#include "nrf_drv_config.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "nrf_error.h"
#include "spi_master.h"

#include "app_scheduler.h"

#include "bsp.h"

#include "treqio_ble.h"
#include "treqio_device.h"
#include "treqio_gps.h"
#include "treqio_gsm.h"
#include "treqio_device_types.h"
#include "treqio_memory.h"

#ifdef DEBUG_SEGGER
#include "SEGGER_RTT.h"
#endif

extern nrf_drv_twi_t twi;
extern treqio_system_config_t treqioConfig;
extern treqio_system_status_t treqioStatus;

uint32_t nrf_gpio_cfg_open_collector(uint8_t pin) {
	NRF_GPIO->PIN_CNF[pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	return NRF_SUCCESS;
}

void treqio_gsm_status_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
	switch (action) {
		case NRF_GPIOTE_POLARITY_HITOLO:
			//do something
			// LEDS_ON(LED_1);
			nrf_gpio_pin_set (LED_BLUE);
			break;
		case NRF_GPIOTE_POLARITY_LOTOHI:
			nrf_gpio_pin_clear(LED_BLUE);
			break;
		default:
			//do nothing
			break;
	}
}

static void bsp_module_init(void) {
	uint32_t err_code;
// Note: If the only use of buttons is to wake up, bsp_event_handler can be NULL.
//    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
//    APP_ERROR_CHECK(err_code);
// Note: If the buttons will be used to do some task, assign bsp_event_handler, as shown below.
//	err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), bsp_event_handler);
	err_code = bsp_init(BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, 0), NULL);
	APP_ERROR_CHECK(err_code);

// You can (if you configured an event handler) choose to assign events to buttons beyond the default configuration.
// E.g:
//  uint32_t err_code = bsp_event_to_button_assign(BUTTON_0_ID, BSP_EVENT_KEY_SLEEP);
//  APP_ERROR_CHECK(err_code);
}

void ADC_IRQHandler(void) {
	nrf_adc_conversion_event_clean();

	uint32_t bat = nrf_adc_result_get();

	bat = ((bat - 470) * 1000) / 281;

	if (bat > 100) {
		bat = 100;
	} else if (bat < 0) {
		bat = 0;
	}

	if (nrf_gpio_pin_read(BATTERY_CHARGE_STATUS) == 1) {
		bat = -bat;
	}

	treqioStatus.batteryLevel = bat;

	nrf_adc_stop();
	nrf_gpio_cfg_input(BATTERY_MEASURE_CTRL, NRF_GPIO_PIN_NOPULL);
}

uint32_t treqio_battery_config_adc() {

	const nrf_adc_config_t nrf_adc_config = { NRF_ADC_CONFIG_RES_10BIT, NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, NRF_ADC_CONFIG_REF_VBG };

	nrf_adc_configure((nrf_adc_config_t *) &nrf_adc_config);
	nrf_adc_input_select (NRF_ADC_CONFIG_INPUT_4);
	nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
	NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
	NVIC_EnableIRQ (ADC_IRQn);

	return NRF_SUCCESS;
}

uint32_t treqio_board_config(void) {
	uint32_t err_code;

	/*
	 * Self powers the device
	 */
//	NRF_GPIO->PIN_CNF[SYSTEM_POWER_CTRL] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
//			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
#if 1
//	nrf_gpio_pin_set (SYSTEM_POWER_CTRL);
//	nrf_delay_ms(250);
#else
	nrf_gpio_pin_set (SYSTEM_POWER_CTRL);
	if ((NRF_POWER->RESETREAS & POWER_RESETREAS_DOG_Msk) != 0) {
		nrf_gpio_pin_set(SYSTEM_POWER_CTRL);
		NRF_POWER->RESETREAS |= POWER_RESETREAS_DOG_Msk;
	} else {
		nrf_gpio_pin_clear(SYSTEM_POWER_CTRL);
	}
#endif

	nrf_gpio_cfg_open_collector (LED_RED);
	nrf_gpio_pin_set(LED_RED);

	nrf_gpio_cfg_open_collector (LED_GREEN);
	nrf_gpio_pin_set(LED_GREEN);

	nrf_gpio_cfg_open_collector (LED_BLUE);
	nrf_gpio_pin_set(LED_BLUE);

	err_code = treqio_uart_config();
	APP_ERROR_CHECK(err_code);

#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
	nrf_gpio_cfg_open_collector(GSM_POWER_CTRL);
	nrf_gpio_pin_set (GSM_POWER_CTRL);
#else
	NRF_GPIO->PIN_CNF[GSM_POWER_CTRL] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	nrf_gpio_pin_clear (GSM_POWER_CTRL);
#endif

	NRF_GPIO->PIN_CNF[GSM_POWER_ON] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	nrf_gpio_pin_clear (GSM_POWER_ON);

	treqioStatus.gsmStatus = TREQIO_GSM_OFF;
	treqioStatus.gsmFirstBooting = true;

	NRF_GPIO->PIN_CNF[MEM_POWER_CTRL] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0D1 << GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	nrf_gpio_pin_clear (MEM_POWER_CTRL);

	return NRF_SUCCESS;
}

uint32_t treqio_board_peripherals_init() {

	uint32_t err_code;

	err_code = treqio_spi_config();
	APP_ERROR_CHECK(err_code);

	err_code = treqio_i2c_config();
	APP_ERROR_CHECK(err_code);

	err_code = treqio_eeprom_read_config(&treqioConfig);
	APP_ERROR_CHECK(err_code);

	return NRF_SUCCESS;
}

static uint32_t treqio_uart_config(void) {

#if 1
//	const app_uart_comm_params_t comm_params = {RXD_PIN_NUMBER, SW_UART_TXD, -1, -1, APP_UART_FLOW_CONTROL_DISABLED, false, UART_BAUDRATE_BAUDRATE_Baud115200};
#else
	uint32_t err_code;

	const app_uart_comm_params_t comm_params = {RXD_PIN_NUMBER, SW_UART_TXD, RTS_PIN_NUMBER, CTS_PIN_NUMBER, APP_UART_FLOW_CONTROL_ENABLED, false,
		UART_BAUDRATE_BAUDRATE_Baud115200};

//	APP_UART_FIFO_INIT(&comm_params, UART_RX_BUFFER_SIZE, UART_RX_BUFFER_SIZE, treqio_uart_event_handler, APP_IRQ_PRIORITY_LOW, err_code);
	APP_UART_INIT(&comm_params, treqio_uart_event_handler, APP_IRQ_PRIORITY_LOW, err_code);
	APP_ERROR_CHECK(err_code);
#endif

	return NRF_SUCCESS;
}

static uint8_t uartRxBuffer[TREQIO_UART_DATA_RX_BUFFER_SIZE];

static uint32_t uartIdx;

void treqio_uart_event_handler(app_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
		case APP_UART_DATA:
		case APP_UART_DATA_READY:
			app_uart_get(&(uartRxBuffer[uartIdx]));
//			SEGGER_RTT_printf(0, "%c", uartRxBuffer[uartIdx]);
			if (uartRxBuffer[uartIdx] == '\n') {
				if (strncmp((char *) &(uartRxBuffer[0]), "Boot", 4) == 0) {
					app_sched_event_put(NULL, 0, treqio_gsm_update_mac_address_event, SCHEDULER_PRIORITY_LOW);
//					if (treqioStatus.gsmStatus <= TREQIO_GSM_FIRST_BOOTING) {
//						treqioStatus.gsmStatus = TREQIO_GSM_FIRST_BOOTED;
//					} else {
//						treqioStatus.gsmStatus = TREQIO_GSM_ON;
//					}
					treqioStatus.gsmStatus = TREQIO_GSM_BOOTED;
				} else if (strncmp((char *) &(uartRxBuffer[0]), "Done", 4) == 0) {
					if (treqioConfig.deviceReportPeriod >= 120000) {
						treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_DONE;
					}
				} else if (strncmp((char *) &(uartRxBuffer[0]), "^SYSSTART", 9) == 0) {
					treqioStatus.gsmStatus = TREQIO_GSM_BOOTING;
				} else if (strncmp((char *) &(uartRxBuffer[0]), "^SYSLOADING", 11) == 0) {
					treqioStatus.gsmStatus = TREQIO_GSM_BOOTING;
				}

				memset(&(uartRxBuffer[0]), 0x00, TREQIO_UART_DATA_RX_BUFFER_SIZE);
//				for (uint32_t idx = 0; idx < uartIdx; idx++) {
//					uartRxBuffer[idx] = ' ';
//				}

				/* Clears expected upload response in case of GSM module reboot */
				treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
				treqioStatus.liveLog.uploadSize = 0;
				treqioStatus.liveMonitor.waiting = false;
				treqioStatus.backlogMonitor.waiting = false;

				uartIdx = 0;
			} else if ((uartRxBuffer[0] == TREQIO_INCOMING_CONFIGURATION_TAG) && (uartIdx > 2)
					&& (uartIdx == ((uartRxBuffer[1] | (uartRxBuffer[2] << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				app_sched_event_put(uartRxBuffer, uartIdx, treqio_process_incoming_configuration_event, SCHEDULER_PRIORITY_LOW);
			} else if ((uartRxBuffer[0] == TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_TAG) && (uartIdx > 2)
					&& (uartIdx == ((uartRxBuffer[1] | (uartRxBuffer[2] << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				app_sched_event_put(uartRxBuffer, uartIdx, treqio_process_incoming_online_ephemeris_size_event, SCHEDULER_PRIORITY_LOW);
			} else if ((uartRxBuffer[0] == TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_TAG) && (uartIdx > 2)
					&& (uartIdx == ((uartRxBuffer[1] | (uartRxBuffer[2] << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				app_sched_event_put(uartRxBuffer, uartIdx, treqio_process_incoming_offline_ephemeris_size_event, SCHEDULER_PRIORITY_LOW);
			} else if ((uartRxBuffer[0] == TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_TAG) && (uartIdx > 2)
					&& (uartIdx == ((uartRxBuffer[1] | (uartRxBuffer[2] << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				app_sched_event_put(&uartRxBuffer, uartIdx, treqio_gps_process_ephemeris_packet_event, SCHEDULER_PRIORITY_LOW);
			} else if ((uartRxBuffer[0] == TREQIO_TAG_DIRECT_UPLOAD) && (uartIdx > 2)
					&& (uartIdx == (((uartRxBuffer[1] & 0xFF) | ((uartRxBuffer[2] & 0xFF) << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				uint16_t size = (uartRxBuffer[3] & 0xFF) | ((uartRxBuffer[4] << 8) & 0xFF);

				treqio_uart_clean_buffer_event(NULL, 0);

				if (size > 0) {
//					treqio_flash_log_delete(treqioStatus.liveLog.uploadStart, size);
//					treqioStatus.liveLog.deleteStart = treqioStatus.liveLog.uploadStart;
//					treqioStatus.liveLog.deleteSize = size;
					app_sched_event_put(NULL, 0, treqio_delete_log_data_event, SCHEDULER_PRIORITY_LOW);
				}

				//				treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
//				treqioStatus.liveLog.uploadSize = 0;
				treqioStatus.liveMonitor.waiting = false;
				app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);

			} else if ((uartRxBuffer[0] == TREQIO_TAG_BLOCK_UPLOAD) && (uartIdx > 2)
					&& (uartIdx == (((uartRxBuffer[1] & 0xFF) | ((uartRxBuffer[2] & 0xFF) << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				uint16_t size = (uartRxBuffer[3] & 0xFF) | ((uartRxBuffer[4] << 8) & 0xFF);

				treqio_uart_clean_buffer_event(NULL, 0);

				if (size > 0) {
#ifdef DEBUG_BACKLOG
					SEGGER_RTT_printf(0, "Backlog block received: %d\r\n", size);
#endif

//					app_sched_event_put(NULL, 0, treqio_gsm_delete_backlog_block_event, SCHEDULER_PRIORITY_LOW);

//					treqioConfig.backLog.start = (treqioConfig.backLog.start + TREQIO_FLASH_LOG_BLOCK_SIZE) & SPI_FLASH_TREQ_LOG_MASK;
//					app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);
				}
//				app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);

				treqioStatus.backlogMonitor.waiting = false;
			} else if ((uartRxBuffer[0] == TREQIO_TAG_STORE_BLOCK) && (uartIdx > 2)
					&& (uartIdx == (((uartRxBuffer[1] & 0xFF) | ((uartRxBuffer[2] & 0xFF) << 8)) + TREQIO_GSM_TAG_HEADER_SIZE))) {
				uint16_t size = (uartRxBuffer[3] & 0xFF) | ((uartRxBuffer[4] << 8) & 0xFF);

				treqio_uart_clean_buffer_event(NULL, 0);

				if (size > 0) {
#ifdef DEBUG_BACKLOG
					SEGGER_RTT_printf(0, "Backlog block received: %d\r\n", size);
#endif
					app_sched_event_put(NULL, 0, treqio_gsm_delete_backlog_block_event, SCHEDULER_PRIORITY_LOW);

//					treqioConfig.backLog.start = (treqioConfig.backLog.start + TREQIO_FLASH_LOG_BLOCK_SIZE) & SPI_FLASH_TREQ_LOG_MASK;
//					app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);
				}
				treqioStatus.backlogMonitor.waiting = false;
			} else {
				uartIdx++;
			}
			break;
		default:
			break;
	}
}

uint32_t treqio_spi_config(void) {
	uint32_t err_code;

	spi_master_config_t spiConfig = SPI_MASTER_INIT_DEFAULT;

	spiConfig.SPI_Pin_SCK = SPIM1_SCK_PIN;
	spiConfig.SPI_Pin_MISO = SPIM1_MISO_PIN;
	spiConfig.SPI_Pin_MOSI = SPIM1_MOSI_PIN;
	spiConfig.SPI_Pin_SS = SPIM1_FLASH_CS_PIN;

	/*
	 * Set SPI bit mode.
	 */
	spiConfig.SPI_CONFIG_ORDER = SPI_MASTER_CONFIG_ORDER;

	/*
	 * Currently at 8Mbit/s.
	 */
	spiConfig.SPI_Freq = SPI_FREQUENCY_FREQUENCY_M8;

	/*
	 * Open SPI as a master device.
	 */
	err_code = spi_master_open(SPI_MASTER, &spiConfig);
	APP_ERROR_CHECK(err_code);

	/*
	 * Register the event handler for this module
	 */
	spi_master_evt_handler_reg(SPI_MASTER, NULL);

	return NRF_SUCCESS;
}

void treqio_spi_event_handler(spi_master_evt_t spi_master_evt) {
//	uint32_t err_code = NRF_SUCCESS;
//	bool result = false;

	switch (spi_master_evt.evt_type) {
		case SPI_MASTER_EVT_TRANSFER_COMPLETED:
//			APP_ERROR_CHECK_BOOL(result);
//			spi_master_close (SPI_MASTER);
			break;

		default:
			break;
	}
}

uint32_t treqio_i2c_config(void) {
	uint32_t err_code;

#if 0
	err_code = nrf_drv_twi_init(&twi, NULL, treqio_i2c_event_handler);
#else
	err_code = nrf_drv_twi_init(&twi, NULL, NULL);
#endif
	APP_ERROR_CHECK(err_code);

//	nrf_delay_ms(250);
	nrf_drv_twi_enable(&twi);
//	nrf_delay_ms(250);
	return NRF_SUCCESS;
}

uint32_t tmp = 0;
uint32_t dados;

void treqio_i2c_event_handler(nrf_drv_twi_evt_t * p_event) {
	nrf_gpio_pin_clear (LED_RED);
	switch (p_event->type) {
		case NRF_DRV_TWI_RX_DONE:
			memcpy(&dados, p_event->p_data, 4);
			break;

		case NRF_DRV_TWI_TX_DONE:
			nrf_drv_twi_rx(&twi, TREQIO_EEPROM_ADDR, (uint8_t *) &tmp, sizeof(uint32_t), false);
			break;

		case NRF_DRV_TWI_ERROR:
			break;

		default:
			break;
	}
}

uint32_t treqio_device_init(treqio_system_config_t * config, treqio_system_status_t * status) {

	status->batteryLevel = 0;
	status->bleStatus = TREQIO_BLE_OFF;
//	status->deviceStatus = TREQIO_DEVICE_ON;
//	memcpy(&(status->gpsLocation), &(config->gpsLastKnownLocation), sizeof(treqio_gps_position_t));
//	status->gpsStatus = TREQIO_GPS_ON;
//	status->gsmStatus = TREQIO_GSM_OFF;
//	status->memoryStatus = TREQIO_MEMORY_OFF;
	status->treqioNewConfig = 0;

	return NRF_SUCCESS;
}

void treqio_device_measure_battery_event(void *p_event_data, uint16_t event_size) {
	nrf_gpio_cfg_output (BATTERY_MEASURE_CTRL);
	nrf_gpio_pin_clear(BATTERY_MEASURE_CTRL);
	nrf_adc_start();
}

void treqio_report_configuration_received_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[64];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	treqio_build_report_tag(&(frame[0]), &frameSize, TREQIO_CONFIG_RECEIVED_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
	frameIdx += frameSize;
	treqio_flash_log_data(&(frame[0]), treqioConfig.backLog.end, TREQIO_INFO_TIMESTAMP_TAG_SIZE, false);
}

void treqio_update_config_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[64];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	treqio_build_report_tag(&(frame[0]), &frameSize, TREQIO_CONFIG_RECEIVED_TAG, NULL, 0);
	frameIdx += frameSize;
	treqio_flash_log_data(&(frame[0]), treqioConfig.backLog.end, TREQIO_INFO_DELTATIME_TAG_SIZE, false);

	treqio_eeprom_write_deviceReportPeriod(&treqioConfig);
	treqio_eeprom_write_gpsSamplingPeriod(&treqioConfig);
	treqioStatus.treqioNewConfig = false;
}

uint32_t treqio_build_report_tag(uint8_t *tag, uint32_t *tagSize, uint8_t type, uint8_t *data, uint32_t dataSize) {
	uint32_t idx = 0;

	switch (type) {
		case (TREQIO_INFO_GPS_LOCATION_TAG): {
			treqio_system_status_t *state = (treqio_system_status_t *) data;
			idx = 0;
			tag[idx++] = TREQIO_INFO_GPS_LOCATION_TAG;
			tag[idx++] = TREQIO_INFO_GPS_LOCATION_SIZE;
			memcpy(&(tag[idx]), &(state->gpsLocation.latitude), TREQIO_INFO_GPS_LATITUDE_SIZE);
			idx += TREQIO_INFO_GPS_LATITUDE_SIZE;
			memcpy(&(tag[idx]), &(state->gpsLocation.longitude), TREQIO_INFO_GPS_LONGITUDE_SIZE);
			idx += TREQIO_INFO_GPS_LONGITUDE_SIZE;
			memcpy(&(tag[idx]), &(state->gpsLocation.altitude), TREQIO_INFO_GPS_ALTITUDE_SIZE);
			idx += TREQIO_INFO_GPS_ALTITUDE_SIZE;
			memcpy(&(tag[idx]), &(state->gpsLocation.dop), TREQIO_INFO_GPS_DILUTION_SIZE);
			idx += TREQIO_INFO_GPS_DILUTION_SIZE;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_INFO_BEACON_TAG): {
			treqio_beacon_info_t *beacon = (treqio_beacon_info_t *) data;
			idx = 0;
			tag[idx++] = TREQIO_INFO_BEACON_TAG;
			tag[idx++] = TREQIO_INFO_BEACON_SIZE;
			memcpy(&(tag[idx]), &(beacon->mac[0]), MAC_ADDRESS_SIZE);
			idx += MAC_ADDRESS_SIZE;
			tag[idx++] = beacon->rssi;
			tag[idx++] = beacon->battery;
			tag[idx++] = beacon->temperature;
			tag[idx++] = beacon->event;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_INFO_GATEWAY_STATUS): {
			treqio_system_status_t *state = (treqio_system_status_t *) data;
			idx = 0;
			tag[idx++] = TREQIO_INFO_GATEWAY_STATUS;
			tag[idx++] = TREQIO_INFO_GATEWAY_STATUS_SIZE;
			tag[idx++] = state->gpsLocation.fix;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_INFO_DELTATIME_TAG): {
			idx = 0;
			tag[idx++] = TREQIO_INFO_DELTATIME_TAG;
			tag[idx++] = TREQIO_INFO_DELTATIME_SIZE;
			memcpy(&(tag[idx]), &(treqioStatus.delta), TREQIO_INFO_DELTATIME_SIZE);
			idx += TREQIO_INFO_DELTATIME_SIZE;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_INFO_TIMESTAMP_TAG): {
			treqio_system_status_t *state = (treqio_system_status_t *) data;

			idx = 0;
			tag[idx++] = TREQIO_INFO_TIMESTAMP_TAG;
			tag[idx++] = TREQIO_INFO_TIMESTAMP_SIZE;

			memcpy(&(tag[idx]), &(state->gpsLocation.time), TREQIO_INFO_TIMESTAMP_SIZE);
			idx += TREQIO_INFO_TIMESTAMP_SIZE;

			memcpy(tagSize, &idx, sizeof(uint32_t));

			break;
		}
		case (TREQIO_INFO_BATTERY_TAG): {
			treqio_system_status_t *state = (treqio_system_status_t *) data;
			idx = 0;
			tag[idx++] = TREQIO_INFO_BATTERY_TAG;
			tag[idx++] = TREQIO_INFO_BATTERY_SIZE;
			tag[idx++] = state->batteryLevel;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_CONFIG_DEVICE_MAC_ADDRESS): {
			idx = 0;
			tag[idx++] = TREQIO_CONFIG_DEVICE_MAC_ADDRESS;
			uint16_t dataSize = TREQIO_CONFIG_DEVICE_MAC_ADDRESS_SIZE;
			memcpy(&(tag[idx]), &dataSize, sizeof(uint16_t));
			idx += sizeof(dataSize);
			memcpy(&(tag[idx]), (void *) 0x100010F0, TREQIO_CONFIG_DEVICE_MAC_ADDRESS_SIZE);
			idx += TREQIO_CONFIG_DEVICE_MAC_ADDRESS_SIZE;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_CONFIG_RECEIVED_TAG): {
			idx = 0;
			tag[idx++] = TREQIO_CONFIG_RECEIVED_TAG;
			tag[idx++] = TREQIO_CONFIG_RECEIVED_TAG_SIZE;
			tag[idx++] = TREQIO_CONFIG_RECEIVED_SUCCESSFUL;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_TAG): {
			idx = 0;
			tag[idx++] = TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_TAG;
			tag[idx++] = TREQIO_REQUEST_OFFLINE_EPHEMERIS_SIZE_SIZE;
			tag[idx++] = TREQIO_GPS_OFFLINE_EPHEMERIS;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_REQUEST_ONLINE_EPHEMERIS_SIZE_TAG): {
			idx = 0;
			tag[idx++] = TREQIO_REQUEST_ONLINE_EPHEMERIS_SIZE_TAG;
			tag[idx++] = TREQIO_REQUEST_ONLINE_EPHEMERIS_SIZE_SIZE;
			tag[idx++] = TREQIO_GPS_ONLINE_EPHEMERIS;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		case (TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_TAG): {
			treqio_system_status_t *state = (treqio_system_status_t *) data;
			idx = 0;
			tag[idx++] = TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_TAG;
			tag[idx++] = TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_SIZE;
			memcpy(&(tag[idx]), &(state->gpsEphemerisPosition), TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_SIZE);
			idx += TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_SIZE;
			memcpy(tagSize, &idx, sizeof(uint32_t));
			break;
		}
		default:
			break;
	}

	return NRF_SUCCESS;
}

//uint32_t treqio_process_incoming_configuration(uint8_t *data, uint8_t size) {

void treqio_process_incoming_configuration_event(void *p_event_data, uint16_t event_size) {

	uint32_t dataIdx = 0;
	treqioStatus.treqioNewConfig = false;

	uint8_t *data = p_event_data;
	uint32_t size = (uint32_t) event_size;

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "New configuration received...\r\n");
#endif

	while (dataIdx < size) {
		switch (data[dataIdx]) {
			case TREQIO_CONFIG_DEVICE_REPORT_RATE: {
				uint32_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.deviceReportPeriod));
#ifdef DEBUG_SEGGER
				SEGGER_RTT_printf(0, "Device Report Period: %d[ms]\r\n", newCfg);
#endif
				if ((newCfg <= TREQIO_CONFIG_DEVICE_REPORT_RATE_MAX) && (newCfg >= TREQIO_CONFIG_DEVICE_REPORT_RATE_MIN)) {
					if (treqioConfig.deviceReportPeriod != newCfg) {
						memcpy(&(treqioConfig.deviceReportPeriod), &newCfg, sizeof(treqioConfig.deviceReportPeriod));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
			case TREQIO_CONFIG_GPS_SAMPLING_RATE: {
				uint32_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.gpsSamplingPeriod));
#ifdef DEBUG_SEGGER
				SEGGER_RTT_printf(0, "GPS Sampling period: %d[ms]\r\n", newCfg);
#endif
				if ((newCfg <= TREQIO_CONFIG_GPS_SAMPLING_RATE_MAX) && (newCfg >= TREQIO_CONFIG_GPS_SAMPLING_RATE_MIN)) {
					if (treqioConfig.gpsSamplingPeriod != newCfg) {
						memcpy(&(treqioConfig.gpsSamplingPeriod), &newCfg, sizeof(treqioConfig.gpsSamplingPeriod));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
			case TREQIO_CONFIG_BLE_SAMPLING_RATE: {
				uint32_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.bleSamplingPeriod));
#ifdef DEBUG_SEGGER
				SEGGER_RTT_printf(0, "BLE Sampling Period: %d[ms]\r\n", newCfg);
#endif
				if ((newCfg <= TREQIO_CONFIG_BLE_SAMPLING_RATE) && (newCfg >= TREQIO_CONFIG_BLE_SAMPLING_RATE_MAX)) {
					if (treqioConfig.bleSamplingPeriod != newCfg) {
						memcpy(&(treqioConfig.bleSamplingPeriod), &newCfg, sizeof(treqioConfig.bleSamplingPeriod));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
			case TREQIO_CONFIG_BLE_SAMPLING_WINDOW: {
				uint32_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.bleSamplingWindow));
#ifdef DEBUG_SEGGER
				SEGGER_RTT_printf(0, "Beacon scan window: %d[ms]\r\n", newCfg);
#endif
				if ((newCfg <= TREQIO_CONFIG_DEVICE_REPORT_RATE_MAX) && (newCfg >= TREQIO_CONFIG_DEVICE_REPORT_RATE_MIN)) {
					if (treqioConfig.bleSamplingWindow != newCfg) {
						memcpy(&(treqioConfig.deviceReportPeriod), &newCfg, sizeof(treqioConfig.deviceReportPeriod));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
			case TREQIO_CONFIG_BLE_BEACON_ALIVE_WINDOW: {
				uint32_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.bleBeaconLifetime));
#ifdef DEBUG_SEGGER
				SEGGER_RTT_printf(0, "Beacon lifetime: %d[ms]\r\n", newCfg);
#endif
				if ((newCfg <= TREQIO_CONFIG_BLE_BEACON_ALIVE_WINDOW_MAX) && (newCfg >= TREQIO_CONFIG_BLE_BEACON_ALIVE_WINDOW_MIN)) {
					if (treqioConfig.bleBeaconLifetime != newCfg) {
						memcpy(&(treqioConfig.bleBeaconLifetime), &newCfg, sizeof(treqioConfig.bleBeaconLifetime));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
			case TREQIO_CONFIG_GPS_EPHEMERIS_TYPE: {
				treqio_gps_ephemeris_type_t newCfg = 0;
				memcpy(&newCfg, &(data[dataIdx + TREQIO_GSM_TAG_HEADER_SIZE]), sizeof(treqioConfig.gpsEphemerisType));
#ifdef DEBUG_SEGGER
				if (newCfg == TREQIO_CONFIG_GPS_EPHEMERIS_TYPE_NO_EPHEMERIS) {
					SEGGER_RTT_printf(0, "%d - No ephemeris.\r\n", newCfg);
				} else if (newCfg == TREQIO_CONFIG_GPS_EPHEMERIS_TYPE_ONLINE) {
					SEGGER_RTT_printf(0, "%d - Online Ephemeris.\r\n", newCfg);
				} else if (newCfg == TREQIO_CONFIG_GPS_EPHEMERIS_TYPE_OFFLINE) {
					SEGGER_RTT_printf(0, "%d - Offline Ephemeris.\r\n", newCfg);
				} else {
					SEGGER_RTT_printf(0, "%d - Invalid ephemeris option.\r\n", newCfg);
				}
#endif
				if ((newCfg <= TREQIO_CONFIG_GPS_EPHEMERIS_TYPE_OFFLINE) && (newCfg >= TREQIO_CONFIG_GPS_EPHEMERIS_TYPE_NO_EPHEMERIS)) {
					if (treqioConfig.gpsEphemerisType != newCfg) {
						memcpy(&(treqioConfig.gpsEphemerisType), &newCfg, sizeof(treqioConfig.gpsEphemerisType));
						treqioStatus.treqioNewConfig = true;
					}
				}
				break;
			}
		}
		dataIdx += data[dataIdx + 1] + 2;
	}

	app_sched_event_put(NULL, 0, treqio_uart_clean_buffer_event, SCHEDULER_PRIORITY_LOW);

	app_sched_event_put(NULL, 0, treqio_report_configuration_received_event, SCHEDULER_PRIORITY_LOW);

//	return NRF_SUCCESS;
}

void treqio_uart_clean_buffer_event(void *p_event_data, uint16_t event_size) {
//	for (uint32_t idx = 0; idx < TREQIO_UART_DATA_RX_BUFFER_SIZE; idx++) {
//		uartRxBuffer[idx] = ' ';
//	}
	memset(&(uartRxBuffer[0]), 0x00, TREQIO_UART_DATA_RX_BUFFER_SIZE);
	uartIdx = 0;
}

#ifdef DEBUG_LIVE_LOG
void treqio_debug_live_log() {
	SEGGER_RTT_printf(0, "\r\n\r\nDebug live log pointers:\r\n");
	SEGGER_RTT_printf(0, "   Live log start: %d\r\n", treqioStatus.liveLog.start);
	SEGGER_RTT_printf(0, "     Live log end: %d\r\n", treqioStatus.liveLog.end);
	SEGGER_RTT_printf(0, "    Live log size: %d\r\n", treqioStatus.liveLog.size);
	SEGGER_RTT_printf(0, "Live upload start: %d\r\n", treqioStatus.liveLog.uploadStart);
	SEGGER_RTT_printf(0, "  Live upload end: %d\r\n", treqioStatus.liveLog.uploadEnd);
	SEGGER_RTT_printf(0, " Live upload size: %d\r\n", treqioStatus.liveLog.uploadSize);
	SEGGER_RTT_printf(0, "\r\n\r\n");
}
#endif

void treqio_delete_log_data_event(void *p_event_data, uint16_t event_size) {
	nrf_gpio_pin_clear (LED_RED);

#ifdef DEBUG_LIVE_LOG
	SEGGER_RTT_printf(0, "Delete Addr: %.8X\r\nSize to Delete: %d\r\n", treqioStatus.liveLog.removeStart, treqioStatus.liveLog.removeSize);
#endif
//	treqio_flash_log_delete(treqioStatus.liveLog.uploadStart, treqioStatus.liveLog.uploadSize);
	treqio_flash_log_delete(treqioStatus.liveLog.removeStart, treqioStatus.liveLog.removeSize);

//	treqio_flash_log_delete(treqioStatus.liveLog.uploadStart, event_size);
}

void treqio_device_reset_timestamp_event(void *p_event_data, uint16_t event_size) {
	treqioStatus.newTimestamp = true;
}

void treqio_device_abort_previous_upload_event(void *p_event_data, uint16_t event_size) {

}

void treqio_device_init_shutdown_event(void *p_event_data, uint16_t event_size) {

	/*
	 * ToDo: Try to send last live log, or save it if not possible as backlog.
	 * ToDo: Try to upload backlog if the live log upload was successful, or there wasn't any live log.
	 */
//	NRF_POWER->GPREGRET = 0xA0;
	if (treqioStatus.liveLog.size > 0) {
		treqioConfig.backLog.end = treqioStatus.liveLog.end;
	} else if (treqioConfig.backLog.start != treqioConfig.backLog.end) {

	}

#ifdef DEBUG_INIT
	SEGGER_RTT_printf(0, "Before Shutting Down...\r\n");
	SEGGER_RTT_printf(0, "Backlog start: %.8X\r\n", treqioConfig.backLog.start);
	SEGGER_RTT_printf(0, "Backlog end: %.8X\r\n", treqioConfig.backLog.end);
	SEGGER_RTT_printf(0, "Livelog start: %.8X\r\n", treqioStatus.liveLog.end);
	SEGGER_RTT_printf(0, "Livelog end: %.8X\r\n", treqioStatus.liveLog.end);
#endif

	treqio_eeprom_write_backlogStart(&treqioConfig);
	treqioConfig.backLog.end = treqioStatus.liveLog.end;
	treqio_eeprom_write_backlogEnd(&treqioConfig);

#ifdef DEBUG_INIT
	SEGGER_RTT_printf(0, "Ready Shutting Down...\r\n");
	SEGGER_RTT_printf(0, "Backlog start: %.8X\r\n", treqioConfig.backLog.start);
	SEGGER_RTT_printf(0, "Backlog end: %.8X\r\n", treqioConfig.backLog.end);
	SEGGER_RTT_printf(0, "Livelog start: %.8X\r\n", treqioStatus.liveLog.end);
	SEGGER_RTT_printf(0, "Livelog end: %.8X\r\n", treqioStatus.liveLog.end);
#endif

	treqioStatus.deviceStatus = TREQIO_DEVICE_OFF;

	app_sched_event_put(NULL, 0, treqio_device_power_off_device, SCHEDULER_PRIORITY_LOW);
}

void treqio_device_power_off_device(void *p_event_data, uint16_t event_size) {
//	NVIC_SystemReset();
	NRF_POWER->GPREGRET = 0xFF;
	NRF_POWER->RESETREAS = 0xFFFFFFFF;
//	nrf_delay_ms(50);
	NVIC_SystemReset();
//	nrf_delay_ms(50);

//	nrf_gpio_pin_clear (SYSTEM_POWER_CTRL);
//	nrf_gpio_pin_set(SYSTEM_POWER_CTRL);
}

