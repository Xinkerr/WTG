#include "sigfox.h"
#include <string.h>
#include <ti/drivers/UART.h>
#include "board.h"
#include "simple_peripheral_oad_offchip.h"

#define COMMS_BOARD_SIGFOX      0x53  // Board type for Sigfox

#define UART_RX_BUF_LEN         (100)  // Receive buffer size
#define UART_SIGFOX_LINE_END    ('\n') // signifies the end of a line from the sigfox board

/* Sigfox regions */
#define SIGFOX_REGION_RCZ1      (0x10)
#define SIGFOX_REGION_RCZ2      (0x20)
#define SIGFOX_REGION_RCZ3      (0x30)
#define SIGFOX_REGION_RCZ4      (0x40)

/* Maximum size of a Sigfox message */
#define SIGFOX_TANK_LEVEL_PAYLOAD_LEN   (12)
#define SIGFOX_TANK_LEVEL_MSG_LEN       (31) // AT$SF=(24 character hex ascii)CRLF

#define SIGFOX_DEVICE_ID_MSG_LEN    (10) // 8 character id + CRLF
#define SIGFOX_PAC_MSG_LEN          (18) // 16 character pac + CRLF

const uint8_t test_msg_buf[] = "AT$SF=53105354\r";

typedef enum {
    SIGFOX_STATE_PRE_INIT,
    SIGFOX_STATE_IDLE,
    SIGFOX_STATE_SLEEP,
    SIGFOX_STATE_REQ_ID,
    SIGFOX_STATE_REQ_PAC,
    SIGFOX_STATE_CHECK_VALS,
    SIGFOX_STATE_SET_RC,
    SIGFOX_STATE_SEND

} sigfox_state_t;

/* Variables */
static volatile char uart_rx_buf[UART_RX_BUF_LEN];
static int uart_rx_index = 0;
static UART_Handle uart = NULL;
static bool sigfox_precheck = false;
static sigfox_state_t sigfox_state = SIGFOX_STATE_PRE_INIT;
static char message[UART_RX_BUF_LEN];
static int message_len = 0;
static uint8_t payload[SIGFOX_TANK_LEVEL_PAYLOAD_LEN];
static int payload_len = 0;
static uint8_t tx_message[SIGFOX_TANK_LEVEL_MSG_LEN];
static int tx_message_len = 0;
static char id[SIGFOX_DEVICE_ID_LEN + 1]; // add null termination
static char pac[SIGFOX_PAC_LEN + 1]; // add null termination

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

	battery_volt: The last byte represents the battery voltage 
			in 20 millivolts increments. For example, the
			value 0xAB in hexadecimal, or 171 in decimal represents 
			3420 millivolts (171 * 20 millivolts).
			The battery voltage is read just before the transmission, 
			after the energy buffering finishes.

	buffer: will hold the encoded sigfox packet with crc and message id
			must be SIGFOX_TANK_LEVEL_LEN bytes (min)

	buffer_len: the length of the buffer
*/
static int sigfox_encode_tank_level_payload(	
	uint32_t timestamp,
	uint32_t level,
	uint8_t status_flags,
	uint8_t battery_volt,
	uint8_t *buffer,
	size_t buffer_len);

/*
	Creates a AT$F=hex_payload formatted message
*/
static size_t sigfox_encode_tank_level_msg(
	const uint8_t *payload, 
	size_t payload_len,
	uint8_t *msg,
	size_t msg_len);

/*
	Converts the 12-bit battery level from <int.frac> 
	to the battery volt format required by sigfox
*/
static uint8_t sigfox_convert_battery_level(uint32_t battery_level);

static void sigfox_check_vals(void);
static void sigfox_set_rc(void);
static void sigfox_send(void);
static void sigfox_sent(void);
static void sigfox_request_pac(void);
static void sigfox_request_id(void);
static void sigfox_got_id(void);
static void sigfox_rx(void);
static void sigfox_on_rx(UART_Handle handle, void *buf, size_t count);
static void sigfox_on_tx(UART_Handle handle, void *buf, size_t count);
static void sigfox_uart_open(void);
static void sigfox_uart_close(void);
static void sigfox_sleep(void);


static uint8_t calculate_crc(uint8_t *buffer, size_t buffer_len);

static uint8_t *to_hex_string(const uint8_t *src, uint8_t src_len,
    uint8_t *dst, uint8_t dst_len);


bool sigfox_init(uint8_t region)
{

    sigfox_uart_open();
    if ((region == SIGFOX_REGION_RCZ2) || (region == SIGFOX_REGION_RCZ4))
    {
        sigfox_precheck = true;
    }

    if (uart != NULL)
    {
        //UART_read(uart, &uart_rx_buf, 1);
        sigfox_request_pac();
        return true;
    }

    return false;
}




void sigfox_log(
	uint32_t timestamp,
	uint32_t level,
	uint8_t status_flags,
	uint32_t battery_level)
{
    if (sigfox_state != SIGFOX_STATE_IDLE)
    {
        return;
    }
    sigfox_uart_open();

    uint8_t battery_volt = sigfox_convert_battery_level(battery_level);
    int ret_val = sigfox_encode_tank_level_payload(
        timestamp, level, status_flags, battery_volt,
        payload, SIGFOX_TANK_LEVEL_PAYLOAD_LEN);

    if (ret_val != SIGFOX_SUCCESS)
    {
        return;
    }

    tx_message_len = sigfox_encode_tank_level_msg(
        payload, SIGFOX_TANK_LEVEL_PAYLOAD_LEN,
        tx_message, SIGFOX_TANK_LEVEL_MSG_LEN);

    if (tx_message_len == 0)
    {
        return;
    }

    if (sigfox_precheck)
    {
        sigfox_check_vals();
        return;
    }

    sigfox_send();
}

void sigfox_tick(void)
{
    switch(sigfox_state)
    {
        case SIGFOX_STATE_CHECK_VALS:
            sigfox_set_rc();
            break;
        case SIGFOX_STATE_SET_RC:
            sigfox_send();
            break;
        case SIGFOX_STATE_SEND:
            sigfox_sent();
            break;
        case SIGFOX_STATE_REQ_PAC:
            sigfox_request_id();
            break;
        case SIGFOX_STATE_REQ_ID:
            sigfox_got_id();
            break;
        case SIGFOX_STATE_SLEEP:
            sigfox_sleep();
            break;
        case SIGFOX_STATE_IDLE:
            break;
        case SIGFOX_STATE_PRE_INIT:
            break;

        default:
            break;
    }

}



static void sigfox_uart_open(void)
{

    UART_Params uartParams;

    if (uart != NULL)
    {
        return;
    }

        UART_init();
        UART_Params_init(&uartParams);
        uartParams.writeDataMode = UART_DATA_TEXT;
        uartParams.writeMode = UART_MODE_CALLBACK;
        uartParams.writeCallback = sigfox_on_tx;
        uartParams.readDataMode = UART_DATA_TEXT;
        uartParams.readReturnMode = UART_RETURN_NEWLINE; // doesn't work
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.readMode = UART_MODE_CALLBACK;
        uartParams.readCallback = sigfox_on_rx;
        uartParams.baudRate = 9600;
        uartParams.dataLength = UART_LEN_8;
        uartParams.parityType = UART_PAR_NONE;
        uartParams.stopBits = UART_STOP_ONE;

        uart = UART_open(Board_UART0, &uartParams);
        if (uart != NULL)
        {
            UART_read(uart, &uart_rx_buf, 1);
        }

}

static void sigfox_uart_close(void)
{
    sigfox_state = SIGFOX_STATE_IDLE;
    if (uart == NULL)
    {
        return;
    }
    UART_readCancel(uart);
    UART_writeCancel(uart);
    UART_close(uart);
    uart = NULL;

}






static int sigfox_encode_tank_level_payload(	
	uint32_t timestamp,
	uint32_t level,
	uint8_t status_flags,
	uint8_t battery_volt,
	uint8_t *buffer,
	size_t buffer_len)
{
	if (buffer_len < SIGFOX_TANK_LEVEL_PAYLOAD_LEN)
	{
		return SIGFOX_BUFFER_ERROR;
	}

	// crc is calculated last
	buffer[1] = SIGFOX_MSG_TANK_LEVEL;

	// time (little endian!)
	buffer[2] = timestamp & 0xFF;
	buffer[3] = (timestamp >> 8) & 0xFF;
	buffer[4] = (timestamp >> 16) & 0xFF;
	buffer[5] = (timestamp >> 24) & 0xFF;
			
	// level (little endian!)
	buffer[6] = level & 0xFF;
	buffer[7] = (level >> 8) & 0xFF;
	buffer[8] = (level >> 16) & 0xFF;
	buffer[9] = (level >> 24) & 0xFF;		

	// status flags, reserved
	buffer[10] = 0x00;

	// battery
	buffer[11] = battery_volt;

	// crc
	buffer[0] = calculate_crc(buffer + 1, SIGFOX_TANK_LEVEL_PAYLOAD_LEN - 1);

	return SIGFOX_SUCCESS;
}

static size_t sigfox_encode_tank_level_msg(
	const uint8_t *payload, 
	size_t payload_len,
	uint8_t *msg,
	size_t msg_len)
{
	uint8_t *ptr = msg;

	if (payload_len < SIGFOX_TANK_LEVEL_PAYLOAD_LEN)
	{
		return SIGFOX_BUFFER_ERROR;
	}	

	if (msg_len < SIGFOX_TANK_LEVEL_MSG_LEN)
	{
		return SIGFOX_BUFFER_ERROR;
	}
	
	*ptr++ = 'A';
	*ptr++ = 'T';
	*ptr++ = '$';
    *ptr++ = 'S';
	*ptr++ = 'F';
	*ptr++ = '=';

	ptr = to_hex_string(payload, payload_len, 
		ptr, SIGFOX_TANK_LEVEL_MSG_LEN - (ptr - msg));

	if (ptr)
	{
		*ptr++ = '\r'; // CR
       // *ptr++ = '\n'; // LF
	}

	// return the number of bytes encoded
	return ptr ? ptr - msg : 0;
}

uint8_t sigfox_convert_battery_level(uint32_t battery_level)
{
	return (uint8_t)(battery_level / 20);
}

static void sigfox_check_vals(void)
{
    sigfox_state = SIGFOX_STATE_CHECK_VALS;
    UART_write(uart, "AT$GI?\r", 7);
}

static void sigfox_set_rc(void)
{
    sigfox_state = SIGFOX_STATE_SET_RC;
    if (message_len != 5)
    {
        sigfox_state = SIGFOX_STATE_SLEEP;
        UART_write(uart, "AT$P=2\r", 7);
        return;
    }

    int x = message[0] - '0';
    int y = message[2] - '0';
    if ((x < 0) || (x > 9) || (y < 0) || (y > 9))
    {
        sigfox_state = SIGFOX_STATE_SLEEP;
        UART_write(uart, "AT$P=2\r", 7);
        return;
    }

    if ((x == 0) || (y < 3))
    {
        UART_write(uart, "AT$RC\r", 6);
    }
    else
    {
        sigfox_send();
    }
}

static void sigfox_send(void)
{
    sigfox_state = SIGFOX_STATE_SEND;
    UART_write(uart, tx_message, tx_message_len);
}

static void sigfox_sent(void)
{
    tx_message_len = 0;
    sigfox_state = SIGFOX_STATE_SLEEP;
    UART_write(uart, "AT$P=2\r", 7);
    //app_enqueueMsg(APP_RX_SIGFOX, NULL, NULL);
    //sigfox_uart_close();
}

static void sigfox_request_pac(void)
{
    sigfox_state = SIGFOX_STATE_REQ_PAC;
    UART_write(uart, "AT$I=11\r", 8);
}

static void sigfox_request_id(void)
{
    sigfox_state = SIGFOX_STATE_REQ_ID;

    // check that we got the PAC and copy it
    if (message_len == SIGFOX_PAC_MSG_LEN)
    {
        memcpy(pac, message, SIGFOX_PAC_LEN);
        pac[SIGFOX_PAC_LEN] = '\0';
    }

    UART_write(uart, "AT$I=10\r", 8);

}

static void sigfox_got_id(void)
{
    sigfox_state = SIGFOX_STATE_SLEEP;

    // check that we got the ID and copy it
    if (message_len == SIGFOX_DEVICE_ID_MSG_LEN)
    {
        memcpy(id, message, SIGFOX_DEVICE_ID_LEN);
        id[SIGFOX_DEVICE_ID_LEN] = '\0';


        app_setCommsBoardDetails(id, pac);
    }
    UART_write(uart, "AT$P=2\r", 7);
}

extern void app_enqueueMsg(uint8_t event, uint8_t state, uint8_t *pData);

static void sigfox_rx(void)
{
    message_len = uart_rx_index;
    if (message_len > sizeof(message))
    {
        message_len = sizeof(message);
    }
    memcpy(message, uart_rx_buf, message_len);
    uart_rx_index = 0;
    app_enqueueMsg(APP_RX_SIGFOX, NULL, NULL);
}

static void sigfox_on_rx(UART_Handle handle, void *buf, size_t count)
{
    if (count)
    {
        uart_rx_index += count;
        if (uart_rx_index > UART_RX_BUF_LEN)
        {
            // panic!!
        } else if ((uart_rx_index == UART_RX_BUF_LEN) || (*(char *)buf == UART_SIGFOX_LINE_END))
        {
            sigfox_rx();
        }
        UART_read(handle, &uart_rx_buf[uart_rx_index], 1);
    }



}


static void sigfox_sleep(void)
{
    sigfox_state = SIGFOX_STATE_IDLE;
    sigfox_uart_close();

}


static void sigfox_on_tx(UART_Handle handle, void *buf, size_t count)
{

}

static uint8_t calculate_crc(uint8_t *buffer, size_t buffer_len)
{
	uint8_t crc = 0x00;
	for (size_t index = 0; index < buffer_len; index++)
	{
		crc += buffer[index];
	}
	return (~crc) + 1;
}

static uint8_t *to_hex_string(
	const uint8_t *src, 
	uint8_t src_len,
    uint8_t *dst, 
    uint8_t dst_len)
{
	char hex[] = "0123456789ABCDEF";
	size_t offset = 0;

	if (dst_len < src_len * 2)
	{
		return NULL;
	}

	memset(dst, 0, dst_len);
	
	for (size_t index = 0; index < src_len; ++index)
	{
		dst[offset] = hex[src[index] >> 4];
		dst[offset + 1] = hex[src[index] & 0x0F];
		offset += 2;
	}

	return dst + offset;
}

/**
 * @brief send message for test
 */
void sigfox_test_send(void)
{
    // sigfox_state = SIGFOX_STATE_SEND;
    // sigfox_uart_open();
    // UART_write(uart, test_msg_buf, sizeof(test_msg_buf));
}
