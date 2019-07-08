/*
 * treqio_gsm.h
 *
 *  Created on: Dec 2, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#ifndef SRC_GSM_TREQIO_GSM_H_
#define SRC_GSM_TREQIO_GSM_H_

#define TMP_TX_BUFFER					128

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "nrf_error.h"

#include "treqio_device.h"

#define TREQIO_TAG_DIRECT_UPLOAD								0x74
#define TREQIO_TAG_BLOCK_UPLOAD									0x44
#define TREQIO_TAG_STORE_BLOCK									0x64
#define TREQIO_TAG_FORCE_UPLOAD_BACKLOG								0x54

uint32_t treqio_gsm_disable(treqio_system_status_t *state);

uint32_t treqio_gsm_enable(treqio_system_status_t *state);

void treqio_gsm_update_mac_address_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_power_up_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_power_on_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_power_on_done_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_report_data_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_report_successfull_data_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_report_battery_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_upload_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_upload_backlog_block_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_delete_backlog_block_event(void *p_event_data, uint16_t event_size);

void treqio_gsm_force_upload_event(void *p_event_data, uint16_t event_size);

#endif /* SRC_GSM_TREQIO_GSM_H_ */
