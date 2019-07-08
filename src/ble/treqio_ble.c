/*
 * treqio_ble.c
 *
 *  Created on: Nov 10, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_uart.h"
#include "ble.h"
#include "ble_advdata.h"

#include "treqio_ble.h"

/**@brief Function to start scanning.
 */
void treqio_ble_scan_start(treqio_system_status_t *status) {
	uint32_t err_code;

	ble_gap_scan_params_t treqio_beacon_scan_parameters; /**< Scan parameters requested for scanning and connection. */

	treqio_beacon_scan_parameters.active = 0; // Active scanning set.
	treqio_beacon_scan_parameters.selective = 0; // Selective scanning not set.
	treqio_beacon_scan_parameters.interval = (uint16_t) SCAN_INTERVAL; // Scan interval.
	treqio_beacon_scan_parameters.window = (uint16_t) SCAN_WINDOW; // Scan window.
	treqio_beacon_scan_parameters.p_whitelist = NULL; // No whitelist provided.
	treqio_beacon_scan_parameters.timeout = 0; // No timeout.

	status->bleStatus = TREQIO_BLE_ON;

	err_code = sd_ble_gap_scan_start(&treqio_beacon_scan_parameters);
	APP_ERROR_CHECK(err_code);

}

//
////static beacon_db_t beacons_db[TREQIO_BEACON_DB_SIZE];
//static volatile uint16_t beacons_db_last_index = 0;
//static volatile uint16_t beacons_db_size = 0;
//
///**@brief Function for handling the Application's BLE Stack events.
// *
// * @param[in]   p_ble_evt   Bluetooth stack event.
// */
//void ble_evt_handler(ble_evt_t *p_ble_evt) {
////	uint32_t err_code;
////	const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
//
//	switch (p_ble_evt->header.evt_id) {
//		case BLE_GAP_EVT_ADV_REPORT: {
////			ble_data_t adv_data;
//
//// Initialize advertisement report for parsing.
////			adv_data.p_data = (uint8_t *) p_gap_evt->params.adv_report.data;
////			adv_data.data_len = p_gap_evt->params.adv_report.dlen;
//
////			uint8_t dbg[64];
////			uint32_t i = 0;
////			app_uart_put(p_gap_evt->params.adv_report.peer_addr.addr_type);
////			for (uint32_t idx = 0; idx < 6; idx++) {
////				app_uart_put(p_gap_evt->params.adv_report.peer_addr.addr[idx]);
////			}
////`			for (uint32_t idx = 0; idx < (adv_data.data_len); idx++) {
////				sscanf(&(dbg[0]), "%2X\r\n", (unsigned int) adv_data.p_data[idx]);
////				printf("%s\n", &(dbg[0]));
////				printf("%.2X\r\n", adv_data.p_data[idx]);
////				app_uart_put(adv_data.p_data[idx]);
////			}
////			break;
//		}
//
////		case BLE_GAP_EVT_TIMEOUT:
////			if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN) {
////				printf("[APPL]: Scan timed out.\r\n");
////				treqio_ble_cfg_scan();
////			} else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
////				printf("[APPL]: Connection Request timed out.\r\n");
////			}
////			break;
////
////		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
////			// Accepting parameters requested by peer.
////			err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle, &p_gap_evt->params.conn_param_update_request.conn_params);
////			APP_ERROR_CHECK(err_code);
////			break;
//
//		default:
//			break;
//	}
//}

//void ble_cfg_scan(void) {
//	// Request creating of whitelist.
//	m_scan_param.active = 0;            // Active scanning set.
//	m_scan_param.selective = 0;            // Selective scanning not set.
//	m_scan_param.interval = SCAN_INTERVAL;            // Scan interval.
//	m_scan_param.window = SCAN_WINDOW;  // Scan window.
//	m_scan_param.p_whitelist = NULL;         // No whitelist provided.
//	m_scan_param.timeout = 0x0000;       // No timeout.
//}
//
//void ble_scan_enable(void) {
//	uint32_t err_code;
//
//	err_code = sd_ble_gap_scan_start(&m_scan_param);
//	APP_ERROR_CHECK(err_code);
//
////	err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
////	APP_ERROR_CHECK(err_code);
//}
//
//void ble_scan_disable(void) {
//	uint32_t err_code;
//
//	err_code = sd_ble_gap_scan_stop();
//	APP_ERROR_CHECK(err_code);
//
////	err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
////	APP_ERROR_CHECK(err_code);
//}
//
//uint32_t beacons_db_add_device(beacon_db_t *beacon) {
//	// Stores ibeacon information into the temporary database if there is still space available.
//	if (beacons_db_size >= TREQIO_BEACON_DB_SIZE) {
//		// Checks if there is space on the database to store one more beacon.
//		return NRF_ERROR_NO_MEM;
//	} else {
//		if (beacons_db_size == beacons_db_last_index) {
//			//Checks if a new slot is required to store the new beacon.
//			memcpy(&(beacons_db[beacons_db_last_index]), beacon, sizeof(beacon_db_t));
//			beacons_db_last_index++;
//			beacons_db_size++;
//		} else {
//			//Uses a previously used slot.
//			uint16_t index = beacons_db_search_empty_slot();
//			memcpy(&(beacons_db[index]), beacon, sizeof(beacon_db_t));
//			beacons_db_size++;
//		}
//	}
//
//	return NRF_SUCCESS;
//}
//
//beacon_db_t beacon_db_get_device(uint16_t index) {
//	beacon_db_t beacon;
//
//	if (beacons_db[index].measures != 0) {
//
//	}
//
//	for (uint8_t idx = 0; idx < TREQIO_BEACON_ID_SIZE; idx++) {
//		beacon.id[idx] = 0;
//	}
//	beacon.rssi = 0;
//	beacon.measures = 0;
//
//	return beacon;
//}
//
//uint32_t beacons_db_remove_device(uint16_t index) {
//	for (uint8_t idx = 0; idx < TREQIO_BEACON_ID_SIZE; idx++) {
//		beacons_db[index].id[idx] = 0;
//	}
//	beacons_db[index].rssi = 0;
//	beacons_db[index].measures = 0;
//
//	if (beacons_db_last_index == beacons_db_size) {
//		while (beacons_db[beacons_db_last_index].measures == 0) {
//			beacons_db_last_index--;
//			beacons_db_size--;
//		}
//	}
//
//	return NRF_SUCCESS;
//}
//
//uint32_t beacons_db_search_device_id(beacon_db_t *beacon, uint16_t *index) {
//
//	if (beacons_db_size == 0) {
//		return NRF_ERROR_NOT_FOUND;
//	} else {
//		for (uint16_t idx = 0; idx < beacons_db_last_index; idx++) {
//			if (memcmp(&(beacons_db[idx]).id, beacon->id, TREQIO_BEACON_ID_SIZE) == 0) {
//				memcpy(index, &idx, sizeof(uint16_t));
//				return NRF_SUCCESS;
//			}
//		}
//	}
//
//	return NRF_ERROR_NOT_FOUND;
//}
//
//uint16_t beacons_db_search_empty_slot() {
//	if (beacons_db_size <= beacons_db_last_index) {
//		for (uint16_t idx = 0; idx < beacons_db_last_index; idx++) {
//			if (beacons_db[idx].measures == 0) {
//				return idx;
//			}
//		}
//	}
//
//	return ((uint16_t) TREQIO_BEACON_DB_SIZE + 1);
//}
