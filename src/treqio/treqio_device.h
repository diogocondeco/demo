/*
 * treqio_device.h
 *
 *  Created on: Dec 1, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#ifndef SRC_TREQIO_TREQIO_DEVICE_H_
#define SRC_TREQIO_TREQIO_DEVICE_H_

#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "spi_master.h"

#include "treqio_ble.h"
#include "treqio_device_types.h"

extern nrf_drv_twi_t twi;

uint32_t nrf_gpio_cfg_open_collector(uint8_t pin);

uint32_t treqio_board_config(void);

static uint32_t treqio_uart_config(void);

void treqio_uart_event_handler(app_uart_evt_t * p_event);

uint32_t treqio_spi_config(void);

void treqio_spi_event_handler(spi_master_evt_t spi_master_evt);

uint32_t treqio_i2c_config(void);

void treqio_i2c_event_handler(nrf_drv_twi_evt_t * p_event);

uint32_t nrf_gpio_cfg_open_collector(uint8_t pin);

void treqio_device_measure_battery_event(void *p_event_data, uint16_t event_size);

void treqio_report_configuration_received_event(void *p_event_data, uint16_t event_size);

void treqio_update_config_event(void *p_event_data, uint16_t event_size);

uint32_t treqio_device_init(treqio_system_config_t *config, treqio_system_status_t *status);

uint32_t treqio_board_peripherals_init();

uint32_t treqio_build_report_tag(uint8_t *tag, uint32_t *tagSize, uint8_t type, uint8_t *data, uint32_t dataSize);

uint32_t treqio_process_incoming_configuration(uint8_t *data, uint8_t size);

void treqio_process_incoming_configuration_event(void *p_event_data, uint16_t event_size);

void treqio_uart_clean_buffer_event(void *p_event_data, uint16_t event_size);

void treqio_debug_live_log();

void treqio_delete_log_data_event(void *p_event_data, uint16_t size);

void treqio_device_reset_timestamp_event(void *p_event_data, uint16_t event_size);

void treqio_device_abort_previous_upload_event(void *p_event_data, uint16_t event_size);

void treqio_device_init_shutdown_event(void *p_event_data, uint16_t event_size);

void treqio_device_power_off_device(void *p_event_data, uint16_t event_size);

#endif /* SRC_TREQIO_TREQIO_DEVICE_H_ */
