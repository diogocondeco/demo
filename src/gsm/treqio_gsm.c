/*
 * treqio_gsm.c
 *
 *  Created on: Jan 18, 2016
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
#include "app_timer.h"
#include "app_uart.h"
#include "app_scheduler.h"

#include "treqio_device.h"
#include "treqio_gps.h"
#include "treqio_gsm.h"
#include "treqio_memory.h"

#ifdef DEBUG_SEGGER
#include "SEGGER_RTT.h"
#endif

//#define TMP_TX_BUFFER			512

extern treqio_system_status_t treqioStatus;
extern treqio_system_config_t treqioConfig;

uint32_t treqio_gsm_disable(treqio_system_status_t *state) {

#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
	nrf_gpio_pin_clear (GSM_POWER_CTRL);
	nrf_gpio_pin_set (GSM_POWER_ON);
#else

	nrf_gpio_pin_clear (GSM_POWER_CTRL);
	nrf_gpio_pin_set (GSM_POWER_ON);
#endif

	treqioStatus.gsmStatus = TREQIO_GSM_OFF;

	return NRF_SUCCESS;
}

uint32_t treqio_gsm_enable(treqio_system_status_t *state) {

//	treqio_gsm_disable(state);
#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
	nrf_gpio_pin_clear (GSM_POWER_CTRL);
#else
	nrf_gpio_pin_set (GSM_POWER_CTRL);
#endif
	treqioStatus.gsmStatus = TREQIO_GSM_POWER_ENABLED;
//	nrf_delay_ms(1000);

//	nrf_gpio_pin_clear (GSM_POWER_ON);
//	nrf_delay_ms(250);

//	nrf_gpio_pin_set(GSM_POWER_ON);

	return NRF_SUCCESS;
}

void treqio_gsm_update_mac_address_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[32];
	uint32_t frameIdx = 0;
	uint32_t frameSize;
	uint8_t checksum = 0;
	treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_CONFIG_DEVICE_MAC_ADDRESS, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
	frameIdx += frameSize;

	for (uint32_t idx = 0; idx < frameIdx; idx++) {
		checksum ^= frame[idx];
		app_uart_put(frame[idx]);
#ifdef DEBUG_MAC
		SEGGER_RTT_printf(0,"0x%.2X ", frame[idx]);
#endif
	}
	app_uart_put(checksum);

#ifdef DEBUG_MAC
	SEGGER_RTT_printf(0, "0x%.2X\r\n", checksum);
#endif

	if (treqioConfig.deviceReportPeriod >= 120000) {
#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
		nrf_gpio_pin_set(GSM_POWER_CTRL);
#else
		nrf_gpio_pin_clear (GSM_POWER_CTRL);
#endif
		treqioStatus.gsmStatus = TREQIO_GSM_OFF;
	} else {
		treqioStatus.gsmStatus = TREQIO_GSM_ON;
	}

#ifdef DEBUG_SEGGER
	SEGGER_RTT_printf(0, "Mac address sent\r\n");
#endif
}

void treqio_gsm_power_up_event(void *p_event_data, uint16_t event_size) {
#ifdef TREQIO_GSM_POWER_INVERTED_LOGIC
	if (nrf_gpio_pin_read(GSM_POWER_CTRL) == 1) nrf_gpio_pin_clear(GSM_POWER_CTRL);
#else
	if (nrf_gpio_pin_read(GSM_POWER_CTRL) == 0) nrf_gpio_pin_set (GSM_POWER_CTRL);
#endif
	treqioStatus.gsmStatus = TREQIO_GSM_POWER_ENABLED;
}

void treqio_gsm_power_on_event(void *p_event_data, uint16_t event_size) {
	if (nrf_gpio_pin_read(GSM_POWER_ON) == 1) nrf_gpio_pin_clear (GSM_POWER_ON);
	treqioStatus.gsmStatus = TREQIO_GSM_POWER_ON;
}

void treqio_gsm_power_on_done_event(void *p_event_data, uint16_t event_size) {
	if (nrf_gpio_pin_read(GSM_POWER_ON) == 0) nrf_gpio_pin_set (GSM_POWER_ON);
	treqioStatus.gsmStatus = TREQIO_GSM_BOOTED;
}

void treqio_gsm_report_data_event(void *p_event_data, uint16_t event_size) {
	app_uart_put('T');
	app_uart_put(0x00);
	app_uart_put(0x54 ^ 0x00);
//	treqioStatus.delta = 0;
}

void treqio_gsm_report_successfull_data_event(void *p_event_data, uint16_t event_size) {
	nrf_gpio_pin_clear (LED_BLUE);
	nrf_delay_ms(50);
	nrf_gpio_pin_set(LED_BLUE);
}

void treqio_gsm_report_battery_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[32];
	uint32_t frameIdx = 0;
	uint32_t frameSize;

	treqio_build_report_tag(&(frame[frameIdx]), &frameSize, TREQIO_INFO_BATTERY_TAG, (uint8_t *) &(treqioStatus), sizeof(treqio_system_status_t));
	frameIdx += frameSize;

//	treqio_flash_log_data(&treqioStatus, &(frame[0]), treqioConfig.logEndIdx, frameIdx);

//	treqioStatus.delta = 0;
	treqioConfig.backLog.upload = treqioConfig.backLog.end;

}

/*
 * Event to upload log from a report event.
 */
void treqio_gsm_upload_event(void *p_event_data, uint16_t event_size) {

//	uint32_t size = treqioConfig.logUploadIdx - treqioConfig.logStartIdx + treqioStatus.logCurrentBlockSize;
//	uint32_t log_idx = 0;
//	uint16_t block_size = 0;

#ifdef DEBUG_SEGGER
//	SEGGER_RTT_printf(0, "Upload event\r\n");
#endif

//	data[0] = 'D';

	treqio_flash_enable(&treqioStatus);

	/*
	 * Process data to send to the GSM Module
	 */

	/*
	 * Test for live data
	 */
	if (treqioStatus.liveLog.size > 0) {

//		if (treqioStatus.liveLog.uploadSize == 0) {
//			treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
//		}
//
//		if ((treqioStatus.liveLog.uploadStart / TREQIO_FLASH_BLOCK_SIZE) != (treqioStatus.liveLog.end / TREQIO_FLASH_BLOCK_SIZE)) {
//			treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
//			treqioConfig.backLog.end = treqioStatus.liveLog.start - 1;
//
//		}

		treqioStatus.liveLog.uploadStart = treqioStatus.liveLog.start;
		treqioStatus.liveLog.uploadEnd = treqioStatus.liveLog.end;
		treqioStatus.liveLog.uploadSize = treqioStatus.liveLog.size;

		treqioStatus.liveLog.removeSize = treqioStatus.liveLog.uploadSize;
		uint32_t frame_size = treqioStatus.liveLog.uploadSize;
//		SEGGER_RTT_printf(0, "Data Size: %d\r\n", frame_size);
//		SEGGER_RTT_printf(0, "Data size: %d\r\n", data_size);

		uint8_t frame[TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE];
		uint32_t frame_idx = 0;
		memset(&(frame[frame_idx]), 0x00, TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE);

		frame[frame_idx++] = TREQIO_TAG_DIRECT_UPLOAD;

		treqio_flash_log_read_live_log(&(frame[TREQIO_GSM_TAG_HEADER_SIZE]), treqioStatus.liveLog.uploadStart, treqioStatus.liveLog.uploadSize);

//		SEGGER_RTT_printf(0, "\r\n\rRaw Info:\r\n");

//		for (uint32_t idx = 0; idx < treqioStatus.liveLog.uploadSize; idx++) {
//			SEGGER_RTT_printf(0, "%.2X ", frame[idx]);
//			nrf_delay_us(50);
//		}

//		SEGGER_RTT_printf(0, "\r\n\r\r\n");

		treqio_flash_log_extract_info_from_block(&(frame[TREQIO_GSM_TAG_HEADER_SIZE]), &frame_size);

//		SEGGER_RTT_printf(0, "Data Size: %d\r\n", frame_size);

		frame[frame_idx++] = (frame_size & 0xFF);
		frame[frame_idx++] = ((frame_size >> 8) & 0xFF);
		frame_idx += frame_size;
//		treqioStatus.liveLog.uploadSize = frame_size;

		treqioStatus.liveLog.removeStart = treqioStatus.liveLog.uploadStart;

//		SEGGER_RTT_printf(0, "Data Size: %d\r\n", frame_idx);

//		for (uint32_t idx = 256; idx < frame_idx; idx++) {
//			frame[idx] = 'U';
//		}

//		app_uart_flush();
		frame[frame_idx++] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + frame_size));

//		if (frame_idx > 255) {
//			frame_idx = 256;
//		}

		app_uart_flush();

		for (uint32_t idx = 0; idx < frame_idx; idx++) {

			app_uart_put(frame[idx]);
//			nrf_delay_us(50);

#ifdef DEBUG_UART_TX
			SEGGER_RTT_printf(0, "%.2x ", frame[idx]);
#endif

		}
	}

	/*
	 * Test for backlog data
	 */
//	if ((treqioConfig.backLog.end != treqioConfig.backLog.start) && (treqioStatus.backlogMonitor.waiting != true)) {
//		app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);
//	}
//
//		uint8_t frame[TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE];
//		uint32_t frame_idx = 0;
//
//		memset(&(frame[0]), 0x00, TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE);
//
//		frame[frame_idx++] = TREQIO_TAG_BLOCK_UPLOAD;
//
//		uint32_t frame_size = 0;
//
//		do {
//			treqio_flash_log_read_backlog_block(&(frame[TREQIO_GSM_TAG_HEADER_SIZE]), &frame_size, treqioConfig.backLog.start);
//			if (frame_size == 0) {
//				treqioConfig.backLog.start = (treqioConfig.backLog.start + TREQIO_FLASH_LOG_BLOCK_SIZE) & SPI_FLASH_TREQ_LOG_MASK;
//			}
//		} while ((frame_size == 0) && (treqioConfig.backLog.end != treqioConfig.backLog.start));
//
//		if (frame_size > 0) {
//			frame[frame_idx++] = (frame_size & 0xFF);
//			frame[frame_idx++] = ((frame_size >> 8) & 0xFF);
//			frame_idx += frame_size;
//
//			frame[frame_idx++] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + frame_size));
//
//			for (uint32_t idx = 0; idx < frame_idx; idx++) {
//				app_uart_put(frame[idx]);
//#ifdef DEBUG_GSM_TRANSFER
//				SEGGER_RTT_printf(0, "%.2x ", frame[idx]);
//#endif
//				nrf_delay_us(100);
//			}
//		}
//	}
	nrf_gpio_pin_set (LED_BLUE);

	treqioStatus.newTimestamp = true;
	treqioStatus.waitingGsmConnection = false;
//	treqioStatus.gsmWaitingDirectUploadResponse = false;

//		uint32_t frame_size = 0;
//		uint8_t frame[TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_FLASH_BLOCK_SIZE];
//		uint32_t frame_idx = 0;

//#ifdef DEBUG_GSM_DATA_UPLOAD
//		SEGGER_RTT_printf(0, "\r\nBacklog data to report...\r\nUploading: ");
//#endif
//		treqioConfig.backLog.upload = treqioConfig.backLog.end;

//		while (treqioConfig.backLog.upload > treqioConfig.backLog.start) {
//
//			data_size = 0;
//			data_idx = 0;
//
//			/*
//			 * ToDo: data[0] can be run only once just ensure memset and other functions are called on data[1]
//			 */
//			memset(&(data[0]), 0x00, (TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_FLASH_BLOCK_SIZE));
//
//			data[data_idx++] = TREQIO_TAG_BLOCK_UPLOAD;
//
//#ifdef DEBUG_GSM_DATA_UPLOAD
//			SEGGER_RTT_printf(0, "%d ", treqioConfig.backLog.start);
//#endif
//			treqio_flash_log_read_backlog_block(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), &data_size, treqioConfig.backLog.start);
//
//			data[data_idx++] = (data_size & 0xFF);
//			data[data_idx++] = ((data_size >> 8) & 0xFF);
//			data_idx += data_size;
//
//			data[data_idx++] = treqio_nmea_checksum_calculator(&(data[0]), (TREQIO_GSM_TAG_OVERHEAD_SIZE + data_size));
//
//			for (uint32_t idx = 0; idx < data_idx; idx++) {
////				app_uart_put(data[idx]);
//			}
//
//			treqioConfig.backLog.start += TREQIO_FLASH_BLOCK_SIZE;
//		}
//	}

	/*
	 * If current block is not full send it via direct upload.
	 */
//#if 1
//	uint8_t data[TREQIO_FLASH_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_WEB_TAG_OVERHEAD_SIZE];
//	data[0] = TREQIO_TAG_DIRECT_UPLOAD;
//	block_size = TREQIO_FLASH_BLOCK_SIZE;
//	memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, sizeof(block_size));
//	uint32_t current_block = treqioConfig.backLog.end / TREQIO_FLASH_BLOCK_SIZE;
//	treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), current_block * TREQIO_FLASH_BLOCK_SIZE, block_size);
//	data[TREQIO_GSM_TAG_HEADER_SIZE + block_size] = treqio_nmea_checksum_calculator(&(data[0]), TREQIO_GSM_TAG_HEADER_SIZE + block_size);
//
////	SEGGER_RTT_printf(0, "\r\n\r\n\r\nData to report: ");
//	for (uint32_t idx = 0; idx < (TREQIO_FLASH_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_WEB_TAG_OVERHEAD_SIZE); idx++) {
////		SEGGER_RTT_printf(0, "%.2X", data[idx]);
////		nrf_delay_us(25);
////	}
//#else
//
//	if ((treqioStatus.logCurrentBlockSize < TREQIO_FLASH_BLOCK_SIZE) && (treqioStatus.logCurrentBlockSize > 0)) {
//
//		uint8_t data[TREQIO_FLASH_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_WEB_TAG_OVERHEAD_SIZE];
//
//		/*
//		 * In the middle of a block.
//		 * Reporting the current block and send a previous one if present.
//		 */
//
//		uint32_t block_start = (treqioConfig.logEndIdx - treqioStatus.logCurrentBlockSize);
//		data[0] = TREQIO_TAG_DIRECT_UPLOAD;
//		block_size = treqioConfig.logEndIdx - block_start;
//		memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, sizeof(block_size));
//
//		treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), block_start, block_size);
//
//		data[TREQIO_GSM_TAG_HEADER_SIZE + block_size] = treqio_nmea_checksum_calculator(&(data[0]), TREQIO_GSM_TAG_HEADER_SIZE + block_size);
//
//#ifdef DEBUG_SEGGER
//		SEGGER_RTT_printf(0, "\r\n\r\n\r\nData to report: ");
//#endif
//		for (uint32_t idx = 0; idx < (TREQIO_GSM_TAG_OVERHEAD_SIZE + block_size); idx++) {
////			app_uart_put(data[idx]);
//#ifdef DEBUG_SEGGER
//			SEGGER_RTT_printf(0, "%.2X",data[idx]);
//#endif
//		}
//
//#ifdef DEBUG_SEGGER
//		SEGGER_RTT_printf(0, "\r\n\r\n");
//#endif
//
//	} else if ((treqioConfig.logUploadIdx - treqioConfig.logStartIdx) > 0) {
//
////		uint8_t data[TREQIO_FLASH_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE + TREQIO_WEB_TAG_OVERHEAD_SIZE];
////
////		data[0] = TREQIO_TAG_BLOCK_UPLOAD;
////
////		if (event_size > 0) {
////
////			block_size = TREQIO_FLASH_BLOCK_SIZE - event_size;
////
////			memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, sizeof(block_size));
////
////			uint32_t block_start = (treqioConfig.logUploadIdx - block_size);
////			treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), block_start, block_size);
////
////		} else {
////			if ((treqioConfig.logUploadIdx - treqioConfig.logStartIdx) > TREQIO_FLASH_BLOCK_SIZE) {
////				block_size = TREQIO_FLASH_BLOCK_SIZE;
////			} else {
////				block_size = (treqioConfig.logUploadIdx - treqioConfig.logStartIdx);
////			}
////
////			memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, sizeof(block_size));
////
////			uint32_t block_start = (treqioConfig.logUploadIdx - TREQIO_FLASH_BLOCK_SIZE);
////			treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), block_start, block_size);
////		}
////
////		data[TREQIO_GSM_TAG_HEADER_SIZE + block_size] = treqio_nmea_checksum_calculator(&(data[0]), TREQIO_GSM_TAG_HEADER_SIZE + block_size);
////
////		for (uint32_t idx = 0; idx < (TREQIO_GSM_TAG_OVERHEAD_SIZE + block_size); idx++) {
////			app_uart_put(data[idx]);
////		}
//	}
//
////	if (treqioConfig.logUploadIdx == treqioConfig.logStartIdx) {
////		treqioConfig.logStartIdx = treqioConfig.logEndIdx - treqioStatus.logCurrentBlockSize;
////		treqioConfig.logUploadIdx = treqioConfig.logStartIdx;
////	}
//
//	/*
//	 * Send a full block.
//	 */
//
////	while (size > log_idx) {
////		block_size = 0;
////		if ((treqioConfig.logStartIdx % TREQIO_FLASH_BLOCK_SIZE) != 0) {
////			if (size >= (treqioConfig.logStartIdx % TREQIO_FLASH_BLOCK_SIZE)) {
////				block_size = TREQIO_FLASH_BLOCK_SIZE - (treqioConfig.logStartIdx % TREQIO_FLASH_BLOCK_SIZE);
////			} else {
////				block_size = size;
////			}
////			treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), treqioConfig.logStartIdx, block_size);
////			memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, TREQIO_GSM_TAG_DATA_SIZE);
////			data[treqioStatus.currentBlockSizeLog + TREQIO_GSM_TAG_HEADER_SIZE] = treqio_nmea_checksum_calculator(&(data[0]),
////					TREQIO_GSM_TAG_HEADER_SIZE + block_size);
////			log_idx += block_size;
////		} else if ((size - log_idx) > TREQIO_FLASH_BLOCK_SIZE) {
////			treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), treqioConfig.logStartIdx + log_idx, TREQIO_FLASH_BLOCK_SIZE);
////			memcpy(&block_size, &(data[TREQIO_GSM_TAG_HEADER_SIZE + TREQIO_FLASH_BLOCK_SIZE - TREQIO_FLASH_BLOCK_SIZE_INFO_SIZE]), TREQIO_GSM_TAG_DATA_SIZE);
////			memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &block_size, TREQIO_GSM_TAG_DATA_SIZE);
////			data[block_size + TREQIO_GSM_TAG_HEADER_SIZE] = treqio_nmea_checksum_calculator(&(data[0]), block_size + TREQIO_GSM_TAG_HEADER_SIZE);
////			log_idx += TREQIO_FLASH_BLOCK_SIZE;
////		} else {
////			block_size = treqioStatus.currentBlockSizeLog;
////			treqio_flash_log_read(&(data[TREQIO_GSM_TAG_HEADER_SIZE]), treqioConfig.logStartIdx + log_idx, treqioStatus.currentBlockSizeLog);
////			memcpy(&(data[TREQIO_GSM_TAG_SIZE]), &treqioStatus.currentBlockSizeLog, TREQIO_GSM_TAG_DATA_SIZE);
////			data[treqioStatus.currentBlockSizeLog + TREQIO_GSM_TAG_HEADER_SIZE] = treqio_nmea_checksum_calculator(&(data[0]),
////					TREQIO_GSM_TAG_HEADER_SIZE + treqioStatus.currentBlockSizeLog);
////			log_idx += treqioStatus.currentBlockSizeLog;
////		}
////
////#ifdef DEBUG_SEGGER
//////		SEGGER_RTT_printf(0, "Memory index: %d\r\n");
////#endif
////
////		for (uint32_t idx = 0; idx < block_size; idx++) {
////#ifdef DEBUG_SEGGER
//////			SEGGER_RTT_printf(0, "%.2X ", data[idx]);
////#endif
////		}
////
////#ifdef DEBUG_SEGGER
//////		SEGGER_RTT_printf(0, "\r\n\r\n\r\n\r\n");
////#endif
////
////	}
////	while (idx < size) {
////		dataIdx = 2;
////		if ((size - idx) > TMP_TX_BUFFER) {
////			data[1] = TMP_TX_BUFFER;
////			treqio_flash_log_read(&treqioStatus, &(data[dataIdx]), treqioConfig.logStartIdx + idx, TMP_TX_BUFFER);
////			dataIdx += TMP_TX_BUFFER;
////			data[dataIdx] = treqio_nmea_checksum_calculator(&(data[0]), dataIdx);
////			dataIdx++;
////			idx += TMP_TX_BUFFER;
////		} else {
////			data[1] = size - idx;
////			treqio_flash_log_read(&treqioStatus, &(data[dataIdx]), treqioConfig.logStartIdx + idx, size - idx);
////			dataIdx += (size - idx);
////			data[dataIdx] = treqio_nmea_checksum_calculator(&(data[0]), dataIdx);
////			dataIdx++;
////			idx = size;
////		}
////		for (uint32_t i = 0; i < dataIdx; i++) {
////			app_uart_put(data[i]);
////#ifdef DEBUG_SEGGER
////			SEGGER_RTT_printf(0, "%.2", data[i]);
////			SEGGER_RTT_printf(0, "\r\n");
////#endif
////		}
////
////	}
////
////#ifdef DEBUG_SEGGER
////	SEGGER_RTT_printf(0, "\r\n\r\n\r\n");
////#endif
////	while (idx < size) {
//////		if ((size - idx) > 31) {
////		data[1] = 31;
////		treqio_flash_log_read(&treqioStatus, &(data[dataIdx]), treqioConfig.logStartIdx + idx, 31);
////		dataIdx += 31;
////		data[dataIdx] = treqio_nmea_checksum_calculator(&(data[0]), dataIdx);
////		dataIdx++;
//////		}
//////		else {
//////			data[1] = size - idx;
//////			treqio_flash_log_read(&treqioStatus, &(data[dataIdx]), treqioConfig.logStartIdx + idx, size - idx);
//////			dataIdx += size - idx;
//////			data[dataIdx] = treqio_nmea_checksum_calculator(&(data[0]), dataIdx);
//////			dataIdx++;
//////		}
////		for (uint32_t i = 0; i < dataIdx; i++) {
////			app_uart_put(data[i]);
////		}
////		dataIdx = 2;
////		idx += 31;
////	}
//#if 0
//	treqio_flash_disable(&treqioStatus);
//	treqioConfig.logStartIdx = treqioConfig.logUploadIdx;
//	app_uart_put('T');
//	app_uart_put(0x00);
//	app_uart_put('T' ^ 0x00);
//#endif
//#endif
}

/*
 * Event to evaluate backlog and upload it if necessary.
 */
void treqio_gsm_upload_backlog(void *p_event_data, uint16_t event_size) {
	if (treqioConfig.deviceReportPeriod >= 120000) {
		treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_IDLE;
	}
}

void treqio_gsm_upload_backlog_block_event(void *p_event_data, uint16_t event_size) {

#ifdef DEBUG_BACKLOG
	SEGGER_RTT_printf(0, "Backlog start: %.8X\r\n", treqioConfig.backLog.start);
	SEGGER_RTT_printf(0, "  Backlog end: %.8X\r\n", treqioConfig.backLog.end);
	SEGGER_RTT_printf(0, "Backlog status: %d\r\n", treqioStatus.backlogMonitor.waiting);
#endif

	if ((treqioConfig.backLog.end != treqioConfig.backLog.start) && (treqioStatus.backlogMonitor.waiting == false)
			&& (treqioStatus.gsmStatus > TREQIO_GSM_BOOTING)) {
		treqioStatus.backlogMonitor.waiting = true;
		uint32_t err_code = app_timer_cnt_get(&(treqioStatus.backlogMonitor.ticks));
		APP_ERROR_CHECK(err_code);

		uint8_t frame[TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE];
		uint32_t frame_idx = 0;

		memset(&(frame[0]), 0x00, TREQIO_FLASH_LOG_BLOCK_SIZE + TREQIO_GSM_TAG_OVERHEAD_SIZE);

		frame[frame_idx++] = TREQIO_TAG_STORE_BLOCK;

		uint32_t frame_size = 0;

		while ((frame_size == 0) && (treqioConfig.backLog.end != treqioConfig.backLog.start)) {
			treqio_flash_log_read_backlog_block(&(frame[TREQIO_GSM_TAG_HEADER_SIZE]), &frame_size, treqioConfig.backLog.start);
			if (frame_size == 0) {
				treqioConfig.backLog.start = (treqioConfig.backLog.start + TREQIO_FLASH_LOG_BLOCK_SIZE) & SPI_FLASH_TREQ_LOG_MASK;
			}
		}

		if (frame_size > 0) {

			frame[frame_idx++] = (frame_size & 0xFF);

			frame[frame_idx++] = ((frame_size >> 8) & 0xFF);
			frame_idx += frame_size;

			frame[frame_idx++] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + frame_size));

			app_uart_flush();

			for (uint32_t idx = 0; idx < frame_idx; idx++) {
				app_uart_put(frame[idx]);
				//				nrf_delay_us(25);
#ifdef DEBUG_GSM_TRANSFER
				SEGGER_RTT_printf(0, "%.2x ", frame[idx]);
#endif
			}
		} else {

#ifdef DEBUG_BACKLOG
			SEGGER_RTT_printf(0, "GSM Command to upload data.\r\n");
#endif

			frame[0] = TREQIO_TAG_FORCE_UPLOAD_BACKLOG;
			frame[1] = 0x01;
			frame[2] = 0x00;
			frame[3] = 0x01;
			frame[4] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + 1));

			for (uint32_t idx = 0; idx < 5; idx++) {
				app_uart_put(frame[idx]);
			}
		}
//	else if (treqioConfig.deviceReportPeriod >= 120000) {
//		treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_IDLE;
//	}
//	} else if (treqioConfig.deviceReportPeriod >= 120000) {
//		treqioStatus.gsmStatus = TREQIO_GSM_COMMUNICATION_IDLE;
	} else if (treqioStatus.backlogMonitor.waiting == true) {
		app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);
	} else {
#ifdef DEBUG_BACKLOG
		SEGGER_RTT_printf(0, "GSM Command to upload data.\r\n");
#endif
		uint8_t frame[5];
		frame[0] = TREQIO_TAG_FORCE_UPLOAD_BACKLOG;
		frame[1] = 0x01;
		frame[2] = 0x00;
		frame[3] = 0x01;
		frame[4] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + 1));

		for (uint32_t idx = 0; idx < 5; idx++) {
			app_uart_put(frame[idx]);
		}

	}

}

void treqio_gsm_delete_backlog_block_event(void *p_event_data, uint16_t event_size) {

	treqio_flash_log_delete(treqioConfig.backLog.start, TREQIO_FLASH_LOG_BLOCK_SIZE);

	treqioConfig.backLog.start = (treqioConfig.backLog.start + TREQIO_FLASH_LOG_BLOCK_SIZE) & SPI_FLASH_TREQ_LOG_MASK;
	treqio_eeprom_write_backlogStart(&treqioConfig);

	app_sched_event_put(NULL, 0, treqio_gsm_upload_backlog_block_event, SCHEDULER_PRIORITY_LOW);

}

void treqio_gsm_force_upload_event(void *p_event_data, uint16_t event_size) {
	uint8_t frame[5];
	frame[0] = TREQIO_TAG_FORCE_UPLOAD_BACKLOG;
	frame[1] = 0x01;
	frame[2] = 0x00;
	frame[3] = 0x01;
	frame[4] = treqio_nmea_checksum_calculator(&(frame[0]), (TREQIO_GSM_TAG_HEADER_SIZE + 1));

	for (uint32_t idx = 0; idx < 5; idx++) {
		app_uart_put(frame[idx]);
	}
}

