/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef TREQIO_H
#define TREQIO_H

// LEDs definitions for TREQIO
#define LEDS_NUMBER    3

#define LED_START       17
#define LED_RED         19
#define LED_GREEN       17
#define LED_BLUE        18
#define LED_STOP        19

#define LEDS_LIST { LED_RED, LED_GREEN, LED_BLUE }

#define BSP_LED_0      LED_RED
#define BSP_LED_1      LED_GREEN
#define BSP_LED_2      LED_BLUE

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK )
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define ACC_EXT_INT					16

#define BATTERY_MEASURE_CTRL		0
#define BATTERY_MEASURE_ADC			3
#define BATTERY_CHARGE_STATUS		21

#define GSM_RESET					4
#define GSM_POWER_CTRL				30
#define GSM_POWER_ON				1
#define GSM_RING					2
#define GSM_STATUS					28

#define GPS_POWER_CTRL				29
#define GPS_BACKUP_CTRL				4
#define GPS_EXT_INT					7

#define DEVICE_BUTTON				10

#define BUTTONS_NUMBER 1

#define BUTTON_START				10
#define BUTTON_1					(DEVICE_BUTTON)
#define BUTTON_STOP					10
#define BUTTON_PULL					NRF_GPIO_PIN_NOPULL

#define BUTTONS_LIST				{ BUTTON_1 }

#define BSP_BUTTON_0				BUTTON_1

#define BSP_BUTTON_0_MASK			(1<<BSP_BUTTON_0)

#define BUTTONS_MASK   				0x001E0000

#define RXD_PIN_NUMBER				22
#define TXD_PIN_NUMBER				24
#define CTS_PIN_NUMBER				23
#define RTS_PIN_NUMBER				25

#define SW_UART_TXD					5
#define SW_UART_RXD					6

#define SYSTEM_POWER_CTRL			20

#define MEM_POWER_CTRL				15

#define TWI_CLK						8
#define TWI_SDA						9

#define UART_RX_BUFFER_SIZE			256
#define UART_TX_BUFFER_SIZE			256

//#define SPIS_MISO_PIN				28    // SPI MISO signal.
//#define SPIS_CSN_PIN				12    // SPI CSN signal.
//#define SPIS_MOSI_PIN				25    // SPI MOSI signal.
//#define SPIS_SCK_PIN				29    // SPI SCK signal.

//#define SPIM0_SCK_PIN				13     /**< SPI clock GPIO pin number. */
//#define SPIM0_MOSI_PIN				14     /**< SPI Master Out Slave In GPIO pin number. */
//#define SPIM0_MISO_PIN				15     /**< SPI Master In Slave Out GPIO pin number. */
//#define SPIM0_SS_PIN				16     /**< SPI Slave Select GPIO pin number. */

#define SPIM1_SCK_PIN				14     /**< SPI clock GPIO pin number. */
#define SPIM1_MOSI_PIN				13     /**< SPI Master Out Slave In GPIO pin number. */
#define SPIM1_MISO_PIN				12     /**< SPI Master In Slave Out GPIO pin number. */
#define SPIM1_FLASH_CS_PIN			11     /**< SPI Slave Select GPIO pin number. */

// serialization APPLICATION board
//#define SER_APP_RX_PIN              12    // UART RX pin number.
//#define SER_APP_TX_PIN              13    // UART TX pin number.
//#define SER_APP_CTS_PIN             15    // UART Clear To Send pin number.
//#define SER_APP_RTS_PIN             14    // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       29     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      25     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      28     // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        12     // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       14     // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       13     // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
//#define SER_CON_RX_PIN              13    // UART RX pin number.
//#define SER_CON_TX_PIN              12    // UART TX pin number.
//#define SER_CON_CTS_PIN             14    // UART Clear To Send pin number. Not used if HWFC is set to false.
//#define SER_CON_RTS_PIN             15    // UART Request To Send pin number. Not used if HWFC is set to false.

//#define SER_CON_SPIS_SCK_PIN        13    // SPI SCK signal.
//#define SER_CON_SPIS_MOSI_PIN       14    // SPI MOSI signal.
//#define SER_CON_SPIS_MISO_PIN       15    // SPI MISO signal.
//#define SER_CON_SPIS_CSN_PIN        16    // SPI CSN signal.
//#define SER_CON_SPIS_RDY_PIN            // SPI READY GPIO pin number.
//#define SER_CON_SPIS_REQ_PIN            // SPI REQUEST GPIO pin number.

#endif // TREQIO_H
