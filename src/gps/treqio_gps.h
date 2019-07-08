/*
 * treqio_gps.h
 *
 *  Created on: Dec 2, 2015
 *      Author: Diogo <diogocondeco@gmail.com>
 */

#ifndef SRC_GPS_TREQIO_GPS_H_
#define SRC_GPS_TREQIO_GPS_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "nrf.h"
#include "nrf_error.h"

#include "treqio_device.h"

#define TREQIO_GPS_I2C_ADDR								0x42

#define TREQIO_GPS_DATA_LENGTH_REG						0xFD
#define TREQIO_GPS_I2C_DATA_REG							0xFF

#define TREQIO_GPS_ONLINE_EPHEMERIS						0x01
#define TREQIO_GPS_OFFLINE_EPHEMERIS					0x02

#define TREQIO_GPS_I2C_BUFFER_SIZE						1024
#define TREQIO_GPS_I2C_FETCH_SIZE						512

#define NMEA_GGA_LATITUDE_INDEX							2
#define NMEA_GGA_LATITUDE_ORIENTATION_INDEX				3
#define NMEA_GGA_LONGITUDE_INDEX						4
#define NMEA_GGA_LONGITUDE_ORIENTATION_INDEX			5
#define NMEA_GGA_NUMBER_SATS_INDEX						7
#define NMEA_GGA_DOP_INDEX 								8
#define NMEA_GGA_ALTITUDE_INDEX							9
#define NMEA_GGA_MEAN_SEA_LEVEL_INDEX					11

#define NMEA_RMC_TIME_INDEX								1
#define NMEA_RMC_DATE_INDEX								9

#define TREQIO_GPS_I2C_RETRIES							10

#define TREQIO_GPS_FIRST_FIX_IGNORE						2

typedef enum {
	TREQIO_GPS_NO_NMEA_SENTENCE = 0, TREQIO_GPS_FULL_NMEA_SENTENCE, TREQIO_GPS_INCOMPLETE_NMEA_SENTENCE,
} treqio_gps_nmea_sentence_t;

extern nrf_drv_twi_t twi;

uint32_t treqio_gps_init(treqio_system_status_t *treqStatus);

uint32_t treqio_gps_power_save_mode(treqio_system_status_t *treqStatus);

uint32_t treqio_gps_positioning_mode(treqio_system_status_t *treqStatus);

uint32_t treqio_gps_shutdown(treqio_system_status_t *treqStatus);

uint32_t treqio_gps_reset(treqio_system_status_t *treqStatus);

treqio_gps_nmea_sentence_t treqio_gps_find_nmea_sentence(char *data, char *nmea, char *result, uint16_t *length);

uint32_t treqio_gps_get_data(uint8_t *data, uint16_t *dataLength, uint16_t *remaining);

static uint32_t treqio_gps_get_info(treqio_gps_position_t *gps);

void treqio_gps_insert_ephemeris(void *p_event_data, uint16_t event_size);

void treqio_gps_sample_data_event(void *p_event_data, uint16_t event_size);

void treqio_gps_store_data_event(void *p_event_data, uint16_t event_size);

uint32_t treqio_nmea_checksum_calculator(uint8_t *data, uint32_t size);

void treqio_gps_process_ephemeris_packet_event(void *p_event_data, uint16_t event_size);

void treqio_gps_retrieve_ephemeris_packet_event(void *p_event_data, uint16_t event_size);

void treqio_process_incoming_online_ephemeris_size_event(void *p_event_data, uint16_t event_size);

void treqio_process_incoming_offline_ephemeris_size_event(void *p_event_data, uint16_t event_size);

#endif /* SRC_GPS_TREQIO_GPS_H_ */
