#ifndef SBP_SIGFOX_SIGFOX_H
#define SBP_SIGFOX_SIGFOX_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SIGFOX_SUCCESS					(0x00)
#define SIGFOX_BUFFER_ERROR				(0x01)

#define SIGFOX_MSG_TANK_LEVEL			(0x10)

#define APP_RX_SIGFOX                   (0x0080)

#define SIGFOX_DEVICE_ID_LEN        (8)
#define SIGFOX_PAC_LEN              (16)

/*
	Sets up the UART connection to the Sigfox module, initialises
	the state machine, and initiates a read with callback.

	Returns true if the UART was able to be opened.

	region: The region code for the module (reaad from i2c)
*/
bool sigfox_init(uint8_t region);

/*
	Creates the tank level payload command as a byte stream
	(binary). The message is sent with an AT command so this
	byte stream should be converted into the transmit buffer
	using encode_tank_level_msg

	timestamp: This is an uint32_t field (4 bytes little-endian) 
			representing the time from the Internal unit
			clock. The time format is Epoch 
			(seconds from 01/Jan/1970 GMT).

	level: This is an uint32_t field (4 bytes little-endian) 
			representing the tank level measure by the sensor.

	status_flags: Reserved

	battery_level: The battery voltage in millivolts.
*/
void sigfox_log(
	uint32_t timestamp,
	uint32_t level,
	uint8_t status_flags,
	uint32_t battery_level);

/*
	Tick function for the Sigfox state machine, called on receipt
	of data on the UART.
*/
void sigfox_tick(void);

/**
 * @brief send message for test
 */
void sigfox_test_send(void);

#ifdef __cplusplus
}
#endif

#endif /* SBP_SIGFOX_SIGFOX_H */
