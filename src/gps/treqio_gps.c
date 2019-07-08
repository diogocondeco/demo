/*
 * treqio_gps.c
 *
 *  Created on: Jan 4, 2016
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "boards.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"

#include "app_scheduler.h"

#include "treqio_device_types.h"
#include "treqio_gps.h"
#include "treqio_gsm.h"
#include "treqio_memory.h"

#ifdef DEBUG_SEGGER
#include "SEGGER_RTT.h"
#endif

extern nrf_drv_twi_t twi;

extern treqio_system_config_t treqioConfig;
extern treqio_system_status_t treqioStatus;

uint8_t treqio_gps_initial_fix_ignore = TREQIO_GPS_FIRST_FIX_IGNORE;

uint32_t treqio_gps_init(treqio_system_status_t *status) {
	nrf_gpio_cfg_output (GPS_POWER_CTRL);
	nrf_gpio_pin_set(GPS_POWER_CTRL);

	nrf_gpio_cfg_output (GPS_BACKUP_CTRL);
	nrf_gpio_pin_set(GPS_BACKUP_CTRL);

	nrf_gpio_cfg_input(GPS_EXT_INT, NRF_GPIO_PIN_NOPULL);

	status->gpsStatus = TREQIO_GPS_OFF;

	return NRF_SUCCESS;
}

uint32_t treqio_gps_power_save_mode(treqio_system_status_t *status) {

	if (status->gpsStatus == TREQIO_GPS_OFF) {
		treqio_gps_init(status);
	}

	nrf_gpio_pin_set (GPS_POWER_CTRL);

//	status->gpsStatus = TREQIO_GPS_SLEEP;

	return NRF_SUCCESS;
}

uint32_t treqio_gps_positioning_mode(treqio_system_status_t *status) {

	if (status->gpsStatus == TREQIO_GPS_OFF) {
		treqio_gps_init(status);
	}

	nrf_gpio_pin_clear (GPS_POWER_CTRL);

	status->gpsStatus = TREQIO_GPS_ON;

	return NRF_SUCCESS;
}

uint32_t treqio_gps_shutdown(treqio_system_status_t *status) {

	if (status->gpsStatus != TREQIO_GPS_OFF) {
		nrf_gpio_pin_set (GPS_POWER_CTRL);
		nrf_gpio_pin_set (GPS_BACKUP_CTRL);
	}

	status->gpsStatus = TREQIO_GPS_OFF;
	status->gpsEphemerisType = TREQIO_GPS_NO_EPHEMERIS;

	return NRF_SUCCESS;
}

uint32_t treqio_gps_reset(treqio_system_status_t *treqStatus) {
	if (treqStatus->gpsStatus != TREQIO_GPS_OFF) {
		nrf_gpio_pin_set (GPS_POWER_CTRL);
		nrf_gpio_pin_set (GPS_BACKUP_CTRL);
	}

	nrf_delay_ms(150);

	if (treqStatus->gpsStatus != TREQIO_GPS_OFF) {
		nrf_gpio_pin_clear (GPS_POWER_CTRL);
		nrf_gpio_pin_clear (GPS_BACKUP_CTRL);
	}

//	treqStatus->gpsStatus = TREQIO_GPS_ON;
	treqStatus->gpsEphemerisType = TREQIO_GPS_NO_EPHEMERIS;

	return NRF_SUCCESS;

}

treqio_gps_nmea_sentence_t treqio_gps_find_nmea_sentence(char *data, char *nmea, char *result, uint16_t *length) {
	char *start = strstr(data, "$");
	uint16_t size = strlen(data);
	char *end = data + size;

	if (start == NULL) {
		return TREQIO_GPS_NO_NMEA_SENTENCE;
	}

	while (size > 0) {
		if (strstr(start, "\n") != NULL) {
			end = strstr(start, "\n") + 1;
			char line[(end - start)];
			strncpy(&(line[0]), start, (end - start));
			line[(end - start)] = '\0';
			if (strstr(line, nmea) != NULL) {
				strcpy(&(result[0]), &(line[0]));
				uint16_t frameSize = (end - start);
				memcpy(length, &frameSize, sizeof(uint16_t));
				return TREQIO_GPS_FULL_NMEA_SENTENCE;
			}
		} else {
			end = data + strlen(data);
			uint16_t frameSize = 0;

			if (strstr(start, ",") != NULL) {
				if (strstr(start, nmea) != NULL) {
					strcpy(&(result[0]), start);
					frameSize = (end - start);
					memcpy(length, &frameSize, sizeof(uint16_t));
					return TREQIO_GPS_INCOMPLETE_NMEA_SENTENCE;
				} else {
					return TREQIO_GPS_NO_NMEA_SENTENCE;
				}
			} else {
				strcpy(&(result[0]), start);
				frameSize = (end - start);
				memcpy(length, &frameSize, sizeof(uint16_t));
				return TREQIO_GPS_INCOMPLETE_NMEA_SENTENCE;
			}
		}

		size -= ((end - start));
		start = end;
	}

	return TREQIO_GPS_NO_NMEA_SENTENCE;
}

char *treqio_gps_isolate_nmea_data(char *gps, uint8_t field) {
	char *idx = gps;
	for (int i = 0; i < field; i++) {
		idx = strchr(idx, ',') + 1;
	}
	return idx;
}

uint32_t treqio_gps_get_location_info(treqio_gps_position_t *location, char *data) {

	int lat = 0;
	int lon = 0;
	int alt = 0;
	uint8_t dop = 0;
	uint8_t sats = 0;

	char str[32];
	char *start;
	char *end;

	if (strstr(data, "$GNGGA,") == NULL) {
		return -1;
	}

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_LATITUDE_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_LATITUDE_INDEX + 1);

	if (start[0] == (start - 1)[0]) {
		return -1;
	}

	strncpy(&(str[0]), start, (end - start));

	lat = atof(str) * 100000;

	if (end[0] == 'S') {
		lat = -lat;
	}

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_LONGITUDE_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_LONGITUDE_INDEX + 1);

	strncpy(&(str[0]), start, (end - start));

	lon = atof(str) * 100000;

	if (end[0] == 'W') {
		lon = -lon;
	}

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_ALTITUDE_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_ALTITUDE_INDEX + 1);

	strncpy(&(str[0]), start, (end - start));

	alt = atof(str) * 1000;

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_MEAN_SEA_LEVEL_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_MEAN_SEA_LEVEL_INDEX + 1);

	strncpy(&(str[0]), start, (end - start));

	alt += atof(str) * 1000;

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_NUMBER_SATS_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_NUMBER_SATS_INDEX + 1);

	strncpy(&(str[0]), start, (end - start));

	sats = atoi(str);

	start = treqio_gps_isolate_nmea_data(data, NMEA_GGA_DOP_INDEX);
	end = treqio_gps_isolate_nmea_data(data, NMEA_GGA_DOP_INDEX + 1);

	strncpy(&(str[0]), start, (end - start));

	dop = atof(str) * 5;

	location->latitude = lat;
	location->longitude = lon;
	location->altitude = alt;
	location->dop = dop;
	location->satellites = sats;

	return NRF_SUCCESS;
}

static uint8_t tags[TREQIO_GPS_I2C_BUFFER_SIZE];
static uint8_t tag[TREQIO_GPS_I2C_FETCH_SIZE];

uint32_t treqio_gps_get_data(uint8_t *data, uint16_t *dataLength, uint16_t *remaining) {

	data = &(tags[0]);
	uint16_t tagLength = 0;
	uint8_t tx[16];
	uint16_t length = 0;
	uint8_t tries = TREQIO_GPS_I2C_RETRIES;

	tx[0] = TREQIO_GPS_DATA_LENGTH_REG;

	while ((nrf_drv_twi_tx(&twi, TREQIO_GPS_I2C_ADDR, &(tx[0]), 1, true) != NRF_SUCCESS) && (tries > 0)) {
		tries--;
	}

	tries = TREQIO_GPS_I2C_RETRIES;
	while ((nrf_drv_twi_rx(&twi, TREQIO_GPS_I2C_ADDR, &(data[0]), 2, false) != NRF_SUCCESS) && (tries > 0)) {
		tries--;
	}

	length = ((((uint32_t) data[0]) << 8) | data[1]);

	tx[0] = TREQIO_GPS_I2C_DATA_REG;

	if (length > 0) {
		if (length < TREQIO_GPS_I2C_FETCH_SIZE) {
			tries = TREQIO_GPS_I2C_RETRIES;
			while ((nrf_drv_twi_tx(&twi, TREQIO_GPS_I2C_ADDR, &(tx[0]), 1, true) != NRF_SUCCESS) && (tries--)) {
				tries--;
			}

			tries = TREQIO_GPS_I2C_RETRIES;
			while ((nrf_drv_twi_rx(&twi, TREQIO_GPS_I2C_ADDR, &(data[0]), (uint32_t) length, false) != NRF_SUCCESS) && (tries > 0)) {
				tries--;
			}

			memcpy(dataLength, &length, sizeof(uint16_t));
			length = 0;
			memcpy(remaining, &length, sizeof(uint16_t));
		} else {
			tries = TREQIO_GPS_I2C_RETRIES;
			while ((nrf_drv_twi_tx(&twi, TREQIO_GPS_I2C_ADDR, &(tx[0]), 1, true) != NRF_SUCCESS) && (tries > 0)) {
				tries--;
			}

			tries = TREQIO_GPS_I2C_RETRIES;
			while ((nrf_drv_twi_rx(&twi, TREQIO_GPS_I2C_ADDR, &(data[0]), TREQIO_GPS_I2C_FETCH_SIZE, false) != NRF_SUCCESS) && (tries > 0)) {
				tries--;
			}
			tagLength = TREQIO_GPS_I2C_FETCH_SIZE;
			memcpy(dataLength, &(tagLength), sizeof(uint16_t));
			length -= TREQIO_GPS_I2C_FETCH_SIZE;
			memcpy(remaining, &length, sizeof(uint16_t));
		}
	}

	return NRF_SUCCESS;
}

uint32_t treqio_gps_get_time_info(treqio_gps_position_t *gps, char *data) {
	struct tm time;
	long long jd = 0;

	char *start;

	if (strstr(data, "$GNRMC,") == NULL) {
		return -1;
	}

	start = treqio_gps_isolate_nmea_data(data, NMEA_RMC_TIME_INDEX);

	if (start == (start + 1)) {
		return -1;
	}

	sscanf(&(start[0]), "%2d", &time.tm_hour);
	sscanf(&(start[2]), "%2d", &time.tm_min);
	sscanf(&(start[4]), "%2d", &time.tm_sec);

	start = treqio_gps_isolate_nmea_data(data, NMEA_RMC_DATE_INDEX);
	sscanf(&(start[0]), "%2d", &time.tm_mday);
	sscanf(&(start[2]), "%2d", &time.tm_mon);
	sscanf(&(start[4]), "%2d", &time.tm_year);

	time.tm_year += 100;
	time.tm_mon -= 1;

	time.tm_isdst = 0;

	jd = ((long long) mktime(&time)) * 1000;

	gps->time = (long long) jd;

	return NRF_SUCCESS;
}

static uint32_t treqio_gps_get_info(treqio_gps_position_t * gps) {

	uint32_t err_code;
	uint16_t tagLength = 0;
	uint16_t length = 0;
	uint16_t remain = 0;
	uint32_t start = 0;

	err_code = treqio_gps_get_data(&(tags[start]), &length, &remain);
	APP_ERROR_CHECK(err_code);

	do {
		switch (treqio_gps_find_nmea_sentence((char *) &(tags[0]), (char *) "$GNGGA", (char *) &(tag[0]), &tagLength)) {
//		switch (treqio_gps_find_nmea_sentence((char *) &(tags[0]), (char *) "$GNGGA", (char *) &(tags[0]), &tagLength)) {
			case TREQIO_GPS_NO_NMEA_SENTENCE: {
				break;
			}

			case TREQIO_GPS_FULL_NMEA_SENTENCE: {
				err_code = treqio_gps_get_location_info(gps, (char *) &(tag[0]));
//				err_code = treqio_gps_get_location_info(gps, (char *) &(tags[0]));
				if (err_code != NRF_SUCCESS) {
					gps->fix = false;
				} else {
					gps->fix = true;
#ifdef DEBUG_SEGGER
//					SEGGER_RTT_printf(0, "Lat: %d\r\nLon: %d\r\nAlt: %d\r\nDop: %d\r\n", gps->latitude, gps->longitude, gps->altitude, gps->dop);
#endif
				}
				break;
			}

			case TREQIO_GPS_INCOMPLETE_NMEA_SENTENCE: {
				memcpy(&(tags[0]), &(tag[0]), tagLength);
//				memcpy(&(tags[0]), &(tags[0]), tagLength);
				start = tagLength;
				break;
			}
		}

		switch (treqio_gps_find_nmea_sentence((char *) &(tags[0]), (char *) "$GNRMC", (char *) &(tag[0]), &tagLength)) {
//		switch (treqio_gps_find_nmea_sentence((char *) &(tags[0]), (char *) "$GNRMC", (char *) &(tags[0]), &tagLength)) {

			case TREQIO_GPS_NO_NMEA_SENTENCE: {
				break;
			}

			case TREQIO_GPS_FULL_NMEA_SENTENCE: {
				err_code = treqio_gps_get_time_info(gps, (char *) &(tag[0]));
//				err_code = treqio_gps_get_time_info(gps, (char *) &(tags[0]));
				break;
			}

			case TREQIO_GPS_INCOMPLETE_NMEA_SENTENCE: {
				memcpy(&(tags[0]), &(tag[0]), tagLength);
//				memcpy(&(tags[0]), &(tags[0]), tagLength);
				start = tagLength;
				break;
			}
		}
		if (remain > 0) {
			err_code = treqio_gps_get_data(&(tags[start]), &length, &remain);
		}
	} while (remain > 0);

	return NRF_SUCCESS;
}

void treqio_gps_insert_ephemeris(void *p_event_data, uint16_t event_size) {

	nrf_gpio_pin_clear (LED_BLUE);
	nrf_delay_ms(100);
	nrf_gpio_pin_set(LED_BLUE);
//	nrf_delay_ms(250);
}

void treqio_gps_sample_data_event(void *p_event_data, uint16_t event_size) {
	treqio_system_status_t tmp;
	treqio_gps_get_info(&(tmp.gpsLocation));

	memcpy(&(treqioStatus.gpsLocation), &(tmp.gpsLocation), sizeof(treqio_gps_position_t));

}

void treqio_gps_store_data_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[TREQIO_FRAME_SIZE];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	memset(&(frame[0]), 0x00, TREQIO_FRAME_SIZE);

	if (treqioStatus.gpsLocation.fix == true) {
		if ((treqioStatus.gpsStatus == TREQIO_GPS_OFF) && (treqio_gps_initial_fix_ignore > 0)) {
			treqio_gps_initial_fix_ignore--;
		} else {
			treqioStatus.gpsStatus = TREQIO_GPS_FIXED;
//		if (treqioStatus.delta == 0) {
//			if (treqioStatus.gsmStatus == TREQIO_GSM_OFF) {
//			}
//			treqio_build_report_tag(&(frame[0]), &frameSize, TREQIO_INFO_TIMESTAMP_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
//			frameIdx += frameSize;
//			treqio_flash_log_data(&(frame[0]), treqioConfig.logEndIdx, TREQIO_INFO_TIMESTAMP_TAG_SIZE, treqioStatus.newReport);
//		}
//			frameIdx = 0;
//			treqioStatus.delta = 0;
//			treqio_build_report_tag(&(frame[0]), &frameSize, TREQIO_INFO_DELTATIME_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_gps_position_t));
//			frameIdx += frameSize;
//			treqio_flash_log_data(&(frame[0]), treqioConfig.logEndIdx, TREQIO_INFO_DELTATIME_TAG_SIZE, treqioStatus.newReport);
			frameIdx = 0;
			treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_GPS_LOCATION_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_gps_position_t));
			frameIdx += frameSize;
			/*
			 * Changed to write on the liveLog.end since all new data is writen on this pointer.
			 */
//			treqio_flash_log_data(&(frame[0]), treqioConfig.backLog.end, TREQIO_INFO_GPS_LOCATION_TAG_SIZE, treqioStatus.newTimestamp);
			treqio_flash_log_data(&(frame[0]), treqioStatus.liveLog.end, TREQIO_INFO_GPS_LOCATION_TAG_SIZE, treqioStatus.newTimestamp);
			if (nrf_gpio_pin_read(DEVICE_BUTTON) != 0) {
				nrf_gpio_pin_set (LED_RED);
				nrf_gpio_pin_clear (LED_GREEN);
				nrf_delay_ms(20);
				nrf_gpio_pin_set(LED_GREEN);
			}
		}
	} else {
		if (treqioStatus.gpsStatus != TREQIO_GPS_OFF) {
			if (treqioStatus.gpsStatus == TREQIO_GPS_FIXED) {
				frameIdx = 0;
				treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_GATEWAY_STATUS, (uint8_t *) &(treqioStatus), sizeof(treqio_gps_position_t));
				frameIdx += frameSize;
				treqio_flash_log_data(&(frame[0]), treqioStatus.liveLog.end, TREQIO_INFO_GATEWAY_STATUS_TAG_SIZE, treqioStatus.newTimestamp);
				treqioStatus.gpsStatus = TREQIO_GPS_LOST_FIX;
			}
			nrf_gpio_pin_clear (LED_RED);
			nrf_delay_ms(20);
			nrf_gpio_pin_set(LED_RED);
		}
	}

	if (treqioStatus.gpsStatus != TREQIO_GPS_OFF) {
//		treqioStatus.delta += treqioConfig.gpsSamplingPeriod;
	}

}

uint32_t treqio_nmea_checksum_calculator(uint8_t *data, uint32_t size) {
	uint32_t checksum = 0;

	for (uint32_t idx = 0; idx < size; idx++) {
		checksum ^= data[idx];
	}

	return checksum;
}

void treqio_gps_process_ephemeris_packet_event(void *p_event_data, uint16_t event_size) {

	uint8_t *data = (uint8_t *) p_event_data;
//	uint32_t size = (uint32_t) event_size;

//	uint8_t tries = TREQIO_GPS_I2C_RETRIES;

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "Ephemeris packet receiced... %d bytes to process...\r\n", event_size);

	for (uint32_t idx = 0; idx < event_size; idx++) {
		SEGGER_RTT_printf(0, "%.2X ", data[idx]);
	}

#endif

//	while ((nrf_drv_twi_rx(&twi, TREQIO_GPS_I2C_ADDR, &(data[0]), (uint32_t) size, false) != NRF_SUCCESS) && (tries > 0)) {
//		tries--;
//	}

	/* Packet sent, buffer can be cleaned to prevent information overlap */
	app_sched_event_put(NULL, 0, treqio_uart_clean_buffer_event, SCHEDULER_PRIORITY_LOW);

	/*
	 * ToDo: Implement the next block request.
	 */

	/*
	 * Evaluate if there are more ephemeris blocks to get.
	 */

	treqioStatus.gpsEphemerisPosition += data[1];

	if (treqioStatus.gpsEphemerisPosition < treqioStatus.gpsEphemerisSize) {
		/*
		 * Request the next ephemeris block
		 */
	} else {
		/*
		 * All blocks of the ephemeris were retrieved.
		 * Ephemeris is ready, sent to the gps and stored for a eventual re-upload.
		 */
	}

}

void treqio_gps_retrieve_ephemeris_packet_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[TREQIO_TEMP_FRAME_SIZE];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	treqio_build_report_tag(&(frame[0]), &frameSize, TREQIO_REQUEST_EPHEMERIS_DATA_BLOCK_TAG, (uint8_t *) &(treqioStatus),
			sizeof(treqioStatus.gpsEphemerisPosition));
	frameIdx += frameSize;

	for (uint32_t idx = 0; idx < frameSize; idx++) {
		app_uart_put(frame[idx]);
	}

}

/*
 * Event to process the offline ephemeris size
 */
void treqio_process_incoming_online_ephemeris_size_event(void *p_event_data, uint16_t event_size) {

	uint8_t *data = (uint8_t *) p_event_data;
	uint32_t size = (uint32_t) event_size;

	if (treqio_nmea_checksum_calculator(data, size - 1) == data[size - 1]) {
		memcpy(&treqioStatus.gpsEphemerisSize, &(data[2]), sizeof(treqioStatus.gpsEphemerisSize));
	}

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "Online ephemeris size: %d[bytes]\r\n", treqioStatus.gpsEphemerisSize);
#endif

	app_sched_event_put(NULL, 0, treqio_uart_clean_buffer_event, SCHEDULER_PRIORITY_LOW);
}

/*
 * Event to process the online ephemeris size
 */
void treqio_process_incoming_offline_ephemeris_size_event(void *p_event_data, uint16_t event_size) {

	uint8_t *data = (uint8_t *) p_event_data;
	uint32_t size = (uint32_t) event_size;

	if (treqio_nmea_checksum_calculator(data, size - 1) == data[size - 1]) {
		memcpy(&treqioStatus.gpsEphemerisSize, &(data[2]), sizeof(treqioStatus.gpsEphemerisSize));
	}

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "Offline ephemeris size: %d[bytes]\r\n", treqioStatus.gpsEphemerisSize);
#endif

	app_sched_event_put(NULL, 0, treqio_uart_clean_buffer_event, SCHEDULER_PRIORITY_LOW);
}

