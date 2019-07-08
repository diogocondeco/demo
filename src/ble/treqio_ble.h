/*
 * treqio_ble.h
 *
 *  Created on: Nov 10, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#ifndef SRC_BLE_TREQIO_BLE_H_
#define SRC_BLE_TREQIO_BLE_H_

#include "ble.h"
#include "ble_advdata.h"

#include "treqio_device_types.h"

/*
 * ToDo: Implement ble scan configuration interval and windows on the protocol
 */
#define SCAN_INTERVAL              0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define TREQIO_BEACON_DB_SIZE			16
#define TREQIO_BEACON_ID_SIZE			6

#define TREQIO_BEACON_MIN_LEVEL			(-100)
#define TREQIO_COMPANY_ID				0x0059
#define TREQIO_COMPANY_ID_POSITION
#define TREQIO_MAJOR_ID					0x0201
#define TREQIO_MAJOR_ID_POSITION		25
#define TREQIO_MINOR_ID					0x0403
#define TREQIO_MINOR_ID_POSITION		27

typedef struct {
	uint8_t * p_data; /**< Pointer to data. */
	uint16_t data_len; /**< Length of data. */
} ble_data_t;

typedef struct {
	uint32_t delta;
	uint32_t timestamp;
	int8_t battery;
	int8_t temperature;
	uint8_t mac[MAC_ADDRESS_SIZE];
	int8_t rssi;
	uint8_t event;
} treqio_beacon_info_t;

void treqio_ble_scan_start(treqio_system_status_t *status);

//void ble_evt_handler(ble_evt_t *p_ble_evt);
//
//void ble_cfg_scan(void);
//
//void ble_scan_enable(void);
//
//void ble_scan_disable(void);

//uint32_t beacons_db_add_device(beacon_db_t *beacon);
//
//beacon_db_t beacon_db_get_device(uint16_t index);
//
//uint32_t beacons_db_remove_device(uint16_t index);
//
//uint32_t beacons_db_search_device_id(beacon_db_t *beacon, uint16_t *index);
//
//uint16_t beacons_db_search_empty_slot();

#endif /* SRC_BLE_TREQIO_BLE_H_ */
