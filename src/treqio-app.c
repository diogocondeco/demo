/*
 * treqio.c
 *
 *  Created on: Nov 3, 2015
 *      Author: Diogo
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "app_error.h"
#include "app_timer_appsh.h"
//#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "ble.h"
#include "boards.h"
#include "bsp.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_config.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_wdt.h"
#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf51_bitfields.h"
#include "softdevice_handler_appsh.h"
#include "spi_master.h"

#include "app_scheduler.h"

#include "treqio_ble.h"
#include "treqio_device.h"
#include "treqio_device_types.h"
#include "treqio_memory.h"
#include "treqio_gps.h"
#include "treqio_gsm.h"

#ifdef USE_BUZZER
#include "app_timer.h"
#include "app_pwm.h"
#include "treqio-pwm.h"
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define SCHEDULER_MAX_EVT_DATA_SIZE				sizeof(app_timer_event_t)
#define SCHEDULER_QUEUE_SIZE					16
#define APP_TIMER_PRESCALER             		0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            		(1 + BSP_APP_TIMERS_NUMBER + 1)				/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE					8
#define TREQIO_SYSTEM_TIMER_PERIOD_MS			100
#define TREQIO_SYSTEM_TIMER_PERIOD				APP_TIMER_TICKS(TREQIO_SYSTEM_TIMER_PERIOD_MS, APP_TIMER_PRESCALER)

#ifdef DEBUG_SEGGER
#include "SEGGER_RTT.h"
#endif

//#define COMPARE_COUNTERTIME  					(3UL)

#ifdef USE_BUZZER
APP_PWM_INSTANCE(PWM1, 2);
app_pwm_config_t pwm1_cfg;
#endif

static treqio_beacon_info_t treqioBeaconsDb[TREQIO_BEACON_DB_SIZE];

static app_timer_id_t treqSystemTimerId;
//static ble_gap_scan_params_t m_scan_param; /**< Scan parameters requested for scanning and connection. */

treqio_system_config_t treqioConfig;
treqio_system_status_t treqioStatus;

nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);
nrf_drv_wdt_config_t wdt = NRF_DRV_WDT_DEAFULT_CONFIG;

uint32_t ledBlinkTicks = 0;
uint32_t ticks = 0;

static void power_manage(void) {
	uint32_t err_code = sd_app_evt_wait();
	APP_ERROR_CHECK(err_code);
}

static void treqio_scheduler_init(void) {
	APP_SCHED_INIT(SCHEDULER_MAX_EVT_DATA_SIZE, SCHEDULER_QUEUE_SIZE);
}

uint32_t treqio_beacon_db_search_beacon(treqio_beacon_info_t *db, treqio_beacon_info_t *beacon, uint32_t *dbIdx) {
	for (uint32_t idx = 0; idx < treqioStatus.bleBeaconsDbSize; idx++) {
		if (db[idx].rssi < 0) {
			if (memcmp(&(db[idx].mac[0]), &(beacon->mac[0]), MAC_ADDRESS_SIZE) == 0) {
//				if (dbIdx != NULL) memcpy(dbIdx, &idx, sizeof(uint32_t));
				return NRF_SUCCESS;
			}
		}
	}
	return NRF_ERROR_NOT_FOUND;
}

void treqio_ble_process_new_beacon_event(void *p_event_data, uint16_t event_size) {

	uint8_t frame[64];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	treqio_beacon_info_t *beacon = &(treqioBeaconsDb[treqioStatus.bleBeaconsDbSize - 1]);

//	SEGGER_RTT_printf(0, "MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\r\n", beacon->mac[0], beacon->mac[1], beacon->mac[2], beacon->mac[3], beacon->mac[4],
//			beacon->mac[5]);

//	nrf_gpio_pin_toggle (LED_RED);

	while (treqioStatus.bleNewBeacons > 0) {
		frameIdx = 0;

		treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_BEACON_TAG, (uint8_t *) beacon, sizeof(treqio_beacon_info_t));
		frameIdx += frameSize;
		treqio_flash_log_data(&(frame[0]), treqioStatus.liveLog.end, TREQIO_INFO_BEACON_TAG_SIZE, false);
		treqioStatus.bleNewBeacons--;
	}

	treqioStatus.bleNewBeacons = 0;
#ifdef USE_BUZZER
	app_pwm_disable (&PWM1);
#endif
}

uint32_t treqio_beacon_db_add_beacon(treqio_beacon_info_t *db, treqio_beacon_info_t *beacon) {
	if ((treqioStatus.bleBeaconsDbSize < TREQIO_BEACON_DB_SIZE) && (treqio_beacon_db_search_beacon(db, beacon, NULL) == NRF_ERROR_NOT_FOUND)
			&& (beacon->rssi > TREQIO_BEACON_MIN_LEVEL)) {
		db[treqioStatus.bleBeaconsDbSize] = *beacon;
		app_sched_event_put(NULL, 0, treqio_ble_process_new_beacon_event, SCHEDULER_PRIORITY_LOW);
		treqioStatus.bleBeaconsDbSize++;
		treqioStatus.bleNewBeacons++;
		return NRF_SUCCESS;
	}
	return NRF_ERROR_NO_MEM;
}

static uint32_t treqio_beacon_db_remove_beacon(treqio_beacon_info_t *db, uint32_t dbIdx) {
	treqio_beacon_info_t emptyBeacon;
	for (uint32_t idx = 0; idx < MAC_ADDRESS_SIZE; idx++) {
		emptyBeacon.mac[idx] = 0x0;
	}
	emptyBeacon.rssi = 0;
	emptyBeacon.battery = 0;
	emptyBeacon.temperature = 0;
	emptyBeacon.event = 0;
	emptyBeacon.timestamp = 0;

	if (treqioStatus.bleBeaconsDbSize > dbIdx) {
		treqioStatus.bleBeaconsDbSize--;
		memcpy(&(db[dbIdx]), &emptyBeacon, sizeof(treqio_beacon_info_t));
		if (treqioStatus.bleBeaconsDbSize > dbIdx) {
			memcpy(&(db[dbIdx]), &(db[dbIdx + 1]), (treqioStatus.bleBeaconsDbSize - dbIdx) * (sizeof(treqio_beacon_info_t)));
			memcpy(&(db[treqioStatus.bleBeaconsDbSize]), &emptyBeacon, sizeof(treqio_beacon_info_t));
		}
		return NRF_SUCCESS;
	}
	return NRF_ERROR_NOT_FOUND;
}

static void treqio_ble_clean_beacons_db(void *p_event_data, uint16_t event_size) {
	uint32_t err_code;
	uint32_t currentTicks = 0;

	err_code = app_timer_cnt_get(&currentTicks);
	APP_ERROR_CHECK(err_code);

	uint32_t delta = 0;

	for (uint32_t idx = 0; idx < treqioStatus.bleBeaconsDbSize; idx++) {
		app_timer_cnt_diff_compute(currentTicks, treqioBeaconsDb[idx].timestamp, &delta);
		if (delta > APP_TIMER_TICKS(treqioConfig.bleBeaconLifetime, APP_TIMER_PRESCALER)) {
			if (treqio_beacon_db_remove_beacon(&(treqioBeaconsDb[0]), idx) == NRF_SUCCESS) {
//				nrf_gpio_pin_toggle (LED_GREEN);
			}
		} else {
			return;
		}
	}
}

void treqio_ble_debug_beacons(void *p_event_data, uint16_t event_size) {
	if (treqioStatus.bleBeaconsDbSize > 0) {
		for (uint32_t idx = 0; idx < treqioStatus.bleBeaconsDbSize; idx++) {
			if (treqioBeaconsDb[idx].rssi != 0x00) {
//				printf("%d beacons\r\n", (int) treqioStatus.bleBeaconsDbSize);
//				printf("[%d]-%.2X%.2X%.2X%.2X%.2X%.2X>%d>%d\r\n", (int) idx, treqioBeaconsDb[idx].mac[0], treqioBeaconsDb[idx].mac[1],
//						treqioBeaconsDb[idx].mac[2], treqioBeaconsDb[idx].mac[3], treqioBeaconsDb[idx].mac[4], treqioBeaconsDb[idx].mac[5],
//						(int) (treqioBeaconsDb[idx].rssi), (int) treqioBeaconsDb[idx].timestamp);
			}
		}
	}
}

void treqio_debug_events(void *p_event_data, uint16_t event_size) {
	nrf_gpio_pin_toggle (LED_BLUE);
//	SEGGER_RTT_printf(0, "Hello World!\r\n");
}

/**@brief Variable length data encapsulation in terms of length and pointer to data */
typedef struct {
	uint8_t *p_data; /**< Pointer to data. */
	uint16_t data_len; /**< Length of data. */
} data_t;

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata) {
	uint32_t index = 0;
	uint8_t * p_data;

	p_data = p_advdata->p_data;

	while (index < p_advdata->data_len) {
		uint8_t field_length = p_data[index];
		uint8_t field_type = p_data[index + 1];

		if (field_type == type) {
			p_typedata->p_data = &p_data[index + 2];
			p_typedata->data_len = field_length - 1;
			return NRF_SUCCESS;
		}
		index += field_length + 1;
	}
	return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt) {
//	uint32_t err_code;
	const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

	switch (p_ble_evt->header.evt_id) {
		case BLE_GAP_EVT_ADV_REPORT: {

			uint16_t majorID = 0xFFFF;
			uint16_t minorID = 0xFFFF;

			memcpy(&(majorID), &(p_ble_evt->evt.gap_evt.params.adv_report.data[TREQIO_MAJOR_ID_POSITION]), sizeof(uint16_t));
			memcpy(&(minorID), &(p_ble_evt->evt.gap_evt.params.adv_report.data[TREQIO_MINOR_ID_POSITION]), sizeof(uint16_t));

//			printf("\r\n");

//			printf("%.2X%.2X == %.4X - %.2X%.2X == %.4X\r\n", p_ble_evt->evt.gap_evt.params.adv_report.data[25],
//					p_ble_evt->evt.gap_evt.params.adv_report.data[26], majorID, p_ble_evt->evt.gap_evt.params.adv_report.data[27],
//					p_ble_evt->evt.gap_evt.params.adv_report.data[28], minorID);

			if ((p_ble_evt->evt.gap_evt.params.adv_report.data[25] == 0x01) && (p_ble_evt->evt.gap_evt.params.adv_report.data[26] == 0x02)
					&& (p_ble_evt->evt.gap_evt.params.adv_report.data[27] == 0x03) && (p_ble_evt->evt.gap_evt.params.adv_report.data[28] == 0x04)) {

				treqio_beacon_info_t beacon;
				//			memcpy(&(beacon.mac[0]), &(p_ble_evt->evt.gap_evt.params.adv_report.peer_addr.addr[0]), MAC_ADDRESS_SIZE);
				memcpy(&(beacon.mac[0]), &(p_gap_evt->params.adv_report.peer_addr.addr[0]), MAC_ADDRESS_SIZE);

#ifdef DEBUG_SEGGER2
				for (uint32_t idx = 0; idx < 31; idx++) {
					SEGGER_RTT_printf(0, "%.2X", p_gap_evt->params.adv_report.data[idx]);
				}
				SEGGER_RTT_printf(0, "\r\n");
				for (uint32_t idx = 0; idx < 6; idx++) {
					SEGGER_RTT_printf(0, "%.2X", beacon.mac[idx]);
				}
				SEGGER_RTT_printf(0, "\r\n");
#endif

				beacon.rssi = p_gap_evt->params.adv_report.rssi; //p_ble_evt->evt.gap_evt.params.adv_report.rssi;
				beacon.battery = p_ble_evt->evt.gap_evt.params.adv_report.data[22];
				beacon.temperature = p_ble_evt->evt.gap_evt.params.adv_report.data[23];
				beacon.event = p_ble_evt->evt.gap_evt.params.adv_report.data[24];

				uint32_t err_code = app_timer_cnt_get(&(beacon.timestamp));
				APP_ERROR_CHECK(err_code);

				if (treqio_beacon_db_add_beacon(&(treqioBeaconsDb[0]), &beacon) == NRF_SUCCESS) {
#ifdef USE_BUZZER
					app_pwm_enable (&PWM1);
#endif
					nrf_gpio_pin_clear (LED_RED);
					nrf_gpio_pin_clear (LED_GREEN);
					nrf_gpio_pin_clear (LED_BLUE);
					nrf_delay_ms(20);
					nrf_gpio_pin_set(LED_RED);
					nrf_gpio_pin_set(LED_GREEN);
					nrf_gpio_pin_set(LED_BLUE);
				} else {

				}
			}
			break;
		}

		case BLE_GAP_EVT_TIMEOUT:
			treqio_ble_scan_start(&treqioStatus);
			break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
			break;

		default:
			break;
	}
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt) {
	on_ble_evt(p_ble_evt);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt) {
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
	uint32_t err_code;

// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

// Enable BLE stack.
	ble_enable_params_t ble_enable_params;
	memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(s132))
	ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
	ble_enable_params.gatts_enable_params.service_changed = false;
#ifdef S120
	ble_enable_params.gap_enable_params.role = BLE_GAP_ROLE_CENTRAL;
#endif

	err_code = sd_ble_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

// Register with the SoftDevice handler module for BLE events.
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);

}

static void treqioSystemTimer(void *p_context) {
	uint32_t err_code;

	/*
	 * Clear Watchdog
	 */
	NRF_WDT->RR[0] = WDT_RR_RR_Reload;

	err_code = app_timer_cnt_get(&ticks);
	APP_ERROR_CHECK(err_code);

	uint32_t diff = 0;

	treqioStatus.delta += TREQIO_SYSTEM_TIMER_PERIOD_MS;

	if (treqioStatus.deviceStatus == TREQIO_DEVICE_ON) {

		if (treqioStatus.liveMonitor.waiting == true) {
			app_timer_cnt_diff_compute(ticks, treqioStatus.liveMonitor.ticks, &diff);
			if (diff > APP_TIMER_TICKS(TREQIO_GSM_OPERATIONS_TIMEOUT_MS, APP_TIMER_PRESCALER)) {
				treqioStatus.liveMonitor.waiting = false;
			}
		}

		if (treqioStatus.backlogMonitor.waiting == true) {
			app_timer_cnt_diff_compute(ticks, treqioStatus.backlogMonitor.ticks, &diff);
			if (diff > APP_TIMER_TICKS(TREQIO_GSM_OPERATIONS_TIMEOUT_MS, APP_TIMER_PRESCALER)) {
				nrf_gpio_pin_clear (LED_GREEN);
				app_sched_event_put(NULL, 0, treqio_gsm_force_upload_event, SCHEDULER_PRIORITY_LOW);
				app_uart_flush();
				treqioStatus.backlogMonitor.waiting = false;
			}
		}

		app_timer_cnt_diff_compute(ticks, APP_TIMER_TICKS(treqioStatus.gpsLastRefresh, APP_TIMER_PRESCALER), &diff);
		if (diff >= APP_TIMER_TICKS(treqioStatus.gpsRefreshRate, APP_TIMER_PRESCALER)) {
			// Sample GPS data
			treqioStatus.gpsLastRefresh += treqioStatus.gpsRefreshRate;
			app_sched_event_put(NULL, 0, treqio_gps_sample_data_event, SCHEDULER_PRIORITY_LOW);
		}

		app_timer_cnt_diff_compute(ticks, APP_TIMER_TICKS(treqioStatus.gpsLastSample, APP_TIMER_PRESCALER), &diff);
		if (diff >= APP_TIMER_TICKS(treqioConfig.gpsSamplingPeriod, APP_TIMER_PRESCALER)) {
			// Store GPS data
			treqioStatus.gpsLastSample += treqioConfig.gpsSamplingPeriod;
			app_sched_event_put(NULL, 0, treqio_gps_store_data_event, SCHEDULER_PRIORITY_LOW);
			app_sched_event_put(NULL, 0, treqio_device_measure_battery_event, SCHEDULER_PRIORITY_LOW);
		}

		/* New upload */
		if (treqioStatus.gsmStatus == TREQIO_GSM_POWER_ENABLED) {
			app_timer_cnt_diff_compute(ticks, APP_TIMER_TICKS(treqioStatus.gsmOnTime, APP_TIMER_PRESCALER), &diff);
			if (diff >= APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)) {
				app_sched_event_put(NULL, 0, treqio_gsm_power_on_event, SCHEDULER_PRIORITY_LOW);
			}
		}

		if ((treqioStatus.gsmStatus == TREQIO_GSM_POWER_ON)) {
			app_timer_cnt_diff_compute(ticks, APP_TIMER_TICKS(treqioStatus.gsmOnTime, APP_TIMER_PRESCALER), &diff);
			if (diff >= APP_TIMER_TICKS(700, APP_TIMER_PRESCALER)) {
				app_sched_event_put(NULL, 0, treqio_gsm_power_on_done_event, SCHEDULER_PRIORITY_LOW);
			}
		}

		/*
		 * Report Period Evaluation
		 * States:
		 * 			-> Report
		 * 			-> No report -> Waiting previous report response
		 * 			-> Nothing
		 */
		app_timer_cnt_diff_compute(ticks, treqioStatus.deviceLastReport, &diff);
		if (diff >= APP_TIMER_TICKS(treqioConfig.deviceReportPeriod, APP_TIMER_PRESCALER)) {
			app_sched_event_put(NULL, 0, treqio_gsm_report_battery_event, SCHEDULER_PRIORITY_LOW);
//			if ((treqioStatus.waitingGsmConnection == false) && (treqioStatus.gpsLocation.fix) && (treqioStatus.gsmWaitingDirectUploadResponse == false)) {
			treqioStatus.deviceLastReport = ticks;
			if (treqioStatus.waitingGsmConnection == false) {
				treqioStatus.waitingGsmConnection = true;
				treqioStatus.gsmOnTime = ticks;
			}

			if (treqioStatus.liveMonitor.waiting == true) {
				treqioStatus.liveMonitor.waiting = false;
			}

			app_sched_event_put(NULL, 0, treqio_device_reset_timestamp_event, SCHEDULER_PRIORITY_LOW);
		}

		if (treqioStatus.gsmFirstBooting == true) {
			/* Device Report through GSM
			 * If off turn on, if on upload data
			 */

			app_timer_cnt_diff_compute(ticks, treqioStatus.gsmOnTime, &diff);
			if (treqioStatus.gsmStatus < TREQIO_GSM_BOOT_POWER_APPLIED) {
#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
				nrf_gpio_pin_clear(GSM_POWER_CTRL);
#else
				nrf_gpio_pin_set (GSM_POWER_CTRL);
#endif
				treqioStatus.gsmStatus = TREQIO_GSM_BOOT_POWER_APPLIED;
			} else if ((diff > 499) && (treqioStatus.gsmStatus < TREQIO_GSM_BOOT_POWER_ON_SIGNAL_APPLIED)) {
				nrf_gpio_pin_clear (GSM_POWER_ON);
				treqioStatus.gsmStatus = TREQIO_GSM_BOOT_POWER_ON_SIGNAL_APPLIED;
			} else if ((diff > 699) && (treqioStatus.gsmStatus < TREQIO_GSM_BOOTING)) {
				nrf_gpio_pin_set (GSM_POWER_ON);
				treqioStatus.gsmStatus = TREQIO_GSM_BOOTING;
			} else if (treqioStatus.gsmStatus > TREQIO_GSM_BOOTING) {

				treqioStatus.gsmFirstBooting = false;
				/*
				 * Implement ephemeris event here
				 */
//				if (treqioStatus.waitingGsmConnection == true) {
//					nrf_gpio_pin_clear (LED_BLUE);
//					app_sched_event_put(NULL, 0, treqio_gsm_upload_event, SCHEDULER_PRIORITY_LOW);
//				}
			}
		} else if (treqioStatus.waitingGsmConnection == true) {
			/* Device Report through GSM
			 * If off turn on, if on upload data
			 */

//			SEGGER_RTT_printf(0, "GSM Initialization State: %d\r\n", treqioStatus.gsmStatus);
//			app_timer_cnt_diff_compute(ticks, treqioStatus.gsmOnTime, &diff);
			if (treqioStatus.gsmStatus < TREQIO_GSM_BOOTED) {
				if ((diff == 0) && (treqioStatus.gsmStatus < TREQIO_GSM_BOOT_POWER_APPLIED)) {
					nrf_gpio_pin_set (GSM_POWER_ON);
					treqioStatus.gsmStatus = TREQIO_GSM_BOOT_POWER_APPLIED;
				} else if ((diff > APP_TIMER_TICKS(499, APP_TIMER_PRESCALER)) && (treqioStatus.gsmStatus < TREQIO_GSM_BOOT_POWER_ON_SIGNAL_APPLIED)) {
					nrf_gpio_pin_clear (GSM_POWER_ON);
					treqioStatus.gsmStatus = TREQIO_GSM_BOOT_POWER_ON_SIGNAL_APPLIED;
				} else if ((diff > APP_TIMER_TICKS(699, APP_TIMER_PRESCALER)) && (treqioStatus.gsmStatus < TREQIO_GSM_BOOTING)) {
					nrf_gpio_pin_set (GSM_POWER_ON);
					treqioStatus.gsmStatus = TREQIO_GSM_BOOTING;
				} else if ((diff > APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER))) {
					treqioStatus.gsmStatus = TREQIO_GSM_OFF;
				}
			} else {
				if (treqioStatus.liveMonitor.waiting == false) {
					nrf_gpio_pin_clear (LED_BLUE);

					app_sched_event_put(NULL, 0, treqio_gsm_upload_event, SCHEDULER_PRIORITY_LOW);
					treqioStatus.liveMonitor.waiting = true;
					err_code = app_timer_cnt_get(&(treqioStatus.liveMonitor.ticks));
					APP_ERROR_CHECK(err_code);
				}
			}
//		}
			if (treqioConfig.deviceReportPeriod >= 120000) {

				if (treqioStatus.gsmStatus == TREQIO_GSM_ON) {
					nrf_gpio_pin_clear (LED_BLUE);
					treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_ACTIVE;
					app_sched_event_put(NULL, 0, treqio_gsm_upload_event, SCHEDULER_PRIORITY_LOW);
				}

				if (treqioStatus.gsmStatus == TREQIO_GSM_COMMUNICATION_DONE) {
					treqio_gsm_disable(&treqioStatus);
				}

				if (treqioStatus.gsmStatus != TREQIO_GSM_OFF) {
					app_timer_cnt_diff_compute(ticks, treqioStatus.gsmOnTime, &diff);
					if (diff >= APP_TIMER_TICKS(90000, APP_TIMER_PRESCALER)) {
						treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_FAILED;
					}
				}

				if (treqioStatus.gsmStatus == TREQIO_GSM_COMMUNICATION_FAILED) {
					treqio_gsm_disable(&treqioStatus);
				}
			}

			if (treqioStatus.treqioNewConfig == true) {
				app_sched_event_put(NULL, 0, treqio_update_config_event, SCHEDULER_PRIORITY_LOW);
			}

		}
	}

	if (treqioStatus.gpsStatus == TREQIO_GPS_FIXED) {
		if (treqioStatus.bleStatus == TREQIO_BLE_OFF) {
			treqio_ble_scan_start(&treqioStatus);
//			nrf_gpio_pin_clear (LED_RED);
		}
	}

	if (treqioStatus.bleStatus == TREQIO_BLE_ON) {
		app_sched_event_put(NULL, 0, treqio_ble_clean_beacons_db, SCHEDULER_PRIORITY_LOW);
//		app_timer_cnt_diff_compute(ticks, APP_TIMER_TICKS(treqioStatus.bleLastSample, APP_TIMER_PRESCALER), &diff);
//		if (diff >= APP_TIMER_TICKS(treqioConfig.bleSamplingPeriod, APP_TIMER_PRESCALER)) {
//			treqio_ble_scan_start();
//		}
//		app_sched_event_put(NULL, 0, treqio_ble_debug_beacons, SCHEDULER_PRIORITY_LOW);
	}

//	app_sched_event_put(NULL, 0, treqio_debug_events, SCHEDULER_PRIORITY_LOW);

}

/**@brief Function for handling a bsp event.
 *
 * @param[in]     evt                        BSP event.
 */
/* YOUR_JOB: Uncomment this function if you need to handle button events. */
//static void bsp_event_handler(bsp_event_t evt) {
//	switch (evt) {
//		default:
//			break;
//	}
//}
static void bsp_module_init(void) {
	uint32_t err_code;
// Note: If the only use of buttons is to wake up, bsp_event_handler can be NULL.
//    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
//    APP_ERROR_CHECK(err_code);
// Note: If the buttons will be used to do some task, assign bsp_event_handler, as shown below.
//	err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), bsp_event_handler);
	err_code = bsp_init(BSP_INIT_BUTTONS, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);

// You can (if you configured an event handler) choose to assign events to buttons beyond the default configuration.
// E.g:
//  uint32_t err_code = bsp_event_to_button_assign(BUTTON_0_ID, BSP_EVENT_KEY_SLEEP);
//  APP_ERROR_CHECK(err_code);
}

//static void timers_init(void) {
//	uint32_t err_code;
//
//	// Initialize timer module, making it use the scheduler
//	APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
//
//	/* YOUR_JOB: Create any timers to be used by the application.
//	 Below is an example of how to create a timer.
//	 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
//	 one. */
//
//}

static void treqioAppTimerInit(void) {
	uint32_t err_code;
	APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
//	APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, NULL);
	err_code = app_timer_create(&treqSystemTimerId, APP_TIMER_MODE_REPEATED, treqioSystemTimer);
	APP_ERROR_CHECK(err_code);
}

static void treqioAppTimerStart(void) {
	uint32_t err_code;

	err_code = app_timer_start(treqSystemTimerId, TREQIO_SYSTEM_TIMER_PERIOD, NULL);
	APP_ERROR_CHECK(err_code);
}

/*
 * NEW BUZZER CODE
 */
#ifdef USE_BUZZER
void pwm_ready_callback(uint32_t pwm_id) {

}

uint32_t treqio_buzzer_pwm_init(uint8_t channel) {

	uint32_t err_code;
//	pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(5000L, channel);
//
	pwm1_cfg.pins[0] = 5;
	pwm1_cfg.pins[1] = APP_PWM_NOPIN;
	pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_LOW;
	pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_LOW;
	pwm1_cfg.num_of_channels = 1;
	pwm1_cfg.period_us = 250;
//
	err_code = app_pwm_init(&PWM1, &pwm1_cfg, pwm_ready_callback);
	APP_ERROR_CHECK(err_code);

	app_pwm_enable (&PWM1);

	while (app_pwm_channel_duty_set(&PWM1, 0, 50) == NRF_ERROR_BUSY)
	;

	app_pwm_disable(&PWM1);

	return NRF_SUCCESS;
}

uint32_t treqio_buzzer_enable(void) {
	app_pwm_enable (&PWM1);

	return NRF_SUCCESS;
}

uint32_t treqio_buzzer_disable(void) {
	app_pwm_disable (&PWM1);

	return NRF_SUCCESS;
}

#endif

/*
 *
 */

int main(void) {

//	treqio_board_config();

	NRF_GPIO->PIN_CNF[SYSTEM_POWER_CTRL] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

	nrf_gpio_pin_set (SYSTEM_POWER_CTRL);

//	while (1);

	treqio_board_config();

	nrf_drv_wdt_init(&wdt, NULL);

	NRF_WDT->RR[0] = WDT_RR_RR_Reload;


	ble_stack_init();
	treqioAppTimerInit();
	treqioAppTimerStart();
	bsp_module_init();
	treqio_scheduler_init();
//	nrf_gpio_cfg_output (LED_RED);
//	nrf_gpio_cfg_output (LED_BLUE);
//	if (nrf_gpio_pin_read(SYSTEM_POWER_CTRL) == 0) {
//	nrf_gpio_pin_clear(LED_BLUE);
//	}
	uint32_t err_code;

	const app_uart_comm_params_t comm_params = { RXD_PIN_NUMBER, TXD_PIN_NUMBER, RTS_PIN_NUMBER, CTS_PIN_NUMBER, APP_UART_FLOW_CONTROL_ENABLED, false,
			UART_BAUDRATE_BAUDRATE_Baud115200 };

	APP_UART_FIFO_INIT(&comm_params, 128, 1024, treqio_uart_event_handler, APP_IRQ_PRIORITY_LOW, err_code);
//	APP_UART_INIT(&comm_params, treqio_uart_event_handler, APP_IRQ_PRIORITY_LOW, err_code);
	APP_ERROR_CHECK(err_code);

//	uint8_t data[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x55, 0x12, 0x55, 0x14, 0x15, 0x16,
//			0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E,
//			0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46,
//			0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E,
//			0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76,
//			0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E,
//			0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6,
//			0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE,
//			0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6,
//			0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0xDF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE,
//			0xEF, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
//			0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x55, 0x12, 0x55, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E,
//			0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
//			0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E,
//			0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66,
//			0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E,
//			0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96,
//			0x97, 0x98, 0x99, 0x9A, 0x9B, 0x9C, 0x9D, 0x9E, 0x9F, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE,
//			0xAF, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6,
//			0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE,
//			0xDF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6,
//			0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF };

//	while (1) {
//		for (uint32_t idx = 0; idx < 512; idx++) {
//			app_uart_put(data[idx]);
////			nrf_delay_ms(1);
//		}
//		nrf_delay_ms(5000);
//	}

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "Treqio booting...\r\n");
#endif

	nrf_gpio_pin_set (MEM_POWER_CTRL);
	treqio_board_peripherals_init();

	/*
	 * Debug
	 */
//
	treqioConfig.backLog.start = 0;
	treqioConfig.backLog.end = 0;
	treqioConfig.backLog.upload = 0;
	treqioStatus.liveLog.start = treqioConfig.backLog.end;
	treqioStatus.liveLog.end = treqioConfig.backLog.end;
	treqioStatus.liveLog.size = 0;
	treqio_eeprom_write_backlogStart(&treqioConfig);
	treqio_eeprom_write_backlogEnd(&treqioConfig);
	treqio_flash_erase(&treqioStatus);
	/*
	 * ENDs here.
	 */

	while ((treqioStatus.deviceStatus == TREQIO_DEVICE_OFF)) {
		app_sched_execute();
	}

//	while (1) {
//		app_sched_execute();
//	}
//	nrf_gpio_pin_set (GPS_POWER_CTRL);

//	treqio_board_peripherals_init();
	treqio_gps_init(&treqioStatus);

	treqio_device_init(&treqioConfig, &treqioStatus);

#ifdef USE_BUZZER
//	nrf_gpio_cfg_output(5);
//	NRF_GPIO->PIN_CNF[5] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_H0D1 << GPIO_PIN_CNF_DRIVE_Pos)
//	| (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
//	| (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
//	nrf_gpio_pin_clear (5);

//	nrf_gpio_pin_clear(5);
	treqio_buzzer_pwm_init(5);
//	NRF_GPIO->PIN_CNF[5] = (GPIO_PIN_CNF_DRIVE_H0D1 << GPIO_PIN_CNF_DRIVE_Pos);
//	app_pwm_enable (&PWM1);
#endif

	treqioStatus.bleNewBeacons = 0;
	treqioStatus.bleLastSample = 0;
	treqioStatus.bleBeaconsDbSize = 0;
	treqioConfig.bleBeaconLifetime = 10000;

	treqioStatus.gpsLocation.latitude = 0;
	treqioStatus.gpsLocation.longitude = 0;
	treqioStatus.gpsLocation.altitude = 0;
	treqioStatus.gpsLocation.dop = 0;
	treqioStatus.gpsLocation.satellites = 0;
	treqioStatus.gpsLocation.fix = false;
	treqioStatus.gpsLocation.time = 0;

	treqioStatus.gpsRefreshRate = 1000;
	treqioStatus.gpsLastRefresh = 0;
	treqioStatus.gpsLastSample = 0;
	treqioStatus.bleLastSample = 0;
	treqioConfig.gpsSamplingPeriod = 1000;
	treqioConfig.deviceReportPeriod = 60000;
	treqioStatus.delta = 0;
	treqioStatus.gsmOnTime = 0;
	treqioStatus.deviceLastReport = 0;
	nrf_gpio_pin_set(MEM_POWER_CTRL);

//	treqio_eeprom_write_deviceReportPeriod(&treqioConfig);
//	treqio_eeprom_write_gpsSamplingPeriod(&treqioConfig);

//	treqio_scheduler_init();

//	treqio_eeprom_read_deviceReportPeriod(&treqioConfig);
//	treqio_eeprom_read_gpsSamplingPeriod(&treqioConfig);

//	treqio_eeprom_read_backlogStart(&treqioConfig);
//	treqio_eeprom_read_backlogEnd(&treqioConfig);
////	treqioConfig.backLog.upload = treqioConfig.backLog.start;
//	treqioConfig.backLog.upload = treqioConfig.backLog.start;
//	treqioStatus.liveLog.start = ((treqioConfig.backLog.end & TREQIO_FLASH_BLOCK_SECTOR_ERASE_ADDR_MASK) + TREQIO_FLASH_BLOCK_SECTOR_ERASE_SIZE)
//			& SPI_FLASH_TREQ_LOG_MASK;
//	if (treqioConfig.backLog.end == treqioConfig.backLog.start) {
//		treqioConfig.backLog.start = treqioStatus.liveLog.start;
//		treqioConfig.backLog.end = treqioStatus.liveLog.start;
//		treqioConfig.backLog.upload = treqioStatus.liveLog.start;
//	} else {
//		treqio_flash_log_delete(treqioConfig.backLog.end, (treqioStatus.liveLog.start - treqioConfig.backLog.end));
//	}
//	treqioStatus.liveLog.end = treqioStatus.liveLog.start;
//	treqioStatus.liveLog.size = 0;
//	treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
//	treqioStatus.liveLog.uploadEnd = treqioStatus.liveLog.start;
//	treqioStatus.liveLog.uploadSize = 0;

	/*
	 * Debug
	 */

//	treqioConfig.backLog.start = 0;
//	treqioConfig.backLog.end = 0;
//	treqioConfig.backLog.upload = 0;
//	treqioStatus.liveLog.start = treqioConfig.backLog.end;
//	treqioStatus.liveLog.end = treqioConfig.backLog.end;
//	treqioStatus.liveLog.size = 0;
//	treqio_eeprom_write_backlogStart(&treqioConfig);
//	treqio_eeprom_write_backlogEnd(&treqioConfig);
//	treqio_flash_erase(&treqioStatus);
	/*
	 * ENDs here.
	 */

	treqioStatus.newTimestamp = true;
	treqioStatus.waitingGsmConnection = false;

	treqioStatus.liveMonitor.waiting = false;
	treqioStatus.liveMonitor.ticks = 0;
	treqioStatus.backlogMonitor.waiting = false;
	treqioStatus.liveMonitor.waiting = 0;

//	nrf_drv_wdt_config_t wdt;
//	wdt.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT;
//	wdt.reload_value = 3000;
//	wdt.interrupt_priority = APP_IRQ_PRIORITY_HIGH;
//	nrf_drv_wdt_init(&wdt, NULL);
//	nrf_drv_wdt_enable();

//	nrf_gpio_pin_clear (LED_BLUE);

//	treqio_flash_erase(&treqioStatus);

#ifdef DEBUG_INIT
	SEGGER_RTT_printf(0, "Sampling Period: %d\r\n", treqioConfig.gpsSamplingPeriod);
	SEGGER_RTT_printf(0, "Report Period: %d\r\n", treqioConfig.deviceReportPeriod);

	SEGGER_RTT_printf(0, "Backlog start: %.8X\r\n", treqioConfig.backLog.start);
	SEGGER_RTT_printf(0, "Backlog end: %.8X\r\n", treqioConfig.backLog.end);
	SEGGER_RTT_printf(0, "Livelog start: %.8X\r\n", treqioStatus.liveLog.end);
	SEGGER_RTT_printf(0, "Livelog end: %.8X\r\n", treqioStatus.liveLog.end);
#endif

//	treqioConfig.backLog.end = (treqioConfig.backLog.end + TREQIO_FLASH_BLOCK_SECTOR_ERASE_SIZE) & 0xFFFFFF000;
//	treqioConfig.backLog.start = treqioConfig.backLog.end;
//	treqioStatus.liveLog.start = treqioConfig.backLog.end;
//	treqioStatus.liveLog.end = treqioConfig.backLog.end;

	//	treqio_flash_erase_block_4k(SPI_FLASH_TREQ_DATA_START + treqioConfig.backLog.end);
//	treqio_flash_erase_block_4k(SPI_FLASH_TREQ_DATA + treqioConfig.backLog.end + 0xC00);
//	treqioStatus.nextLogBlockCleared = true;
	treqioStatus.nextBlockCounter = 0;

	treqio_flash_erase(&treqioStatus);

//	treqio_eeprom_write_deviceReportPeriod(&treqioConfig);
//	treqio_eeprom_write_gpsSamplingPeriod(&treqioConfig);

//	treqio_eeprom_read_deviceReportPeriod(&treqioConfig);
//	treqio_eeprom_read_gpsSamplingPeriod(&treqioConfig);

	treqioStatus.deviceStatus = TREQIO_DEVICE_ON;
//	nrf_gpio_pin_clear (LED_GREEN);

	for (;;) {
		app_sched_execute();
		power_manage();
	}
}
