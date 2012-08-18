#ifndef LINUX_SERIAL_DEV_H
#define LINUX_SERIAL_DEV_H

#define NUM_START_BYTES 2

/* NOTE: be sure to use here the same values as in the firmware */
#define START_BYTE1	'z'
#define START_BYTE2	'b'
#define MAX_DATA_SIZE	127

/* states set by linux */
#define IDLE_MODE	0x00
#define RX_MODE		0x02
#define TX_MODE		0x03
#define FORCE_TRX_OFF	0xF0

#define STATUS_SUCCESS	0
#define STATUS_RX_ON	1
#define STATUS_TX_ON	2
#define STATUS_TRX_OFF	3
#define STATUS_IDLE	4
#define STATUS_BUSY	5
#define STATUS_BUSY_RX	6
#define STATUS_BUSY_TX	7
#define STATUS_ERR	8

/* We re-use PPP ioctl for our purposes */
#define	PPPIOCGUNIT	_IOR('t', 86, int)	/* get ppp unit number */

/*
 * The following messages are used to control ZigBee firmware.
 * All communication has request/response format,
 * except of asynchronous incoming data stream (DATA_RECV_* messages).
 */
enum {
	NO_ID			= 0, /* means no pending id */

	/* Driver to Firmware */
	CMD_OPEN		= 0x01, /* u8 id */
	CMD_CLOSE		= 0x02, /* u8 id */
	CMD_SET_CHANNEL		= 0x04, /* u8 id, u8 channel */
	CMD_ED			= 0x05, /* u8 id */
	CMD_CCA			= 0x06, /* u8 id */
	CMD_SET_STATE		= 0x07, /* u8 id, u8 flag */
	DATA_XMIT_BLOCK		= 0x09, /* u8 id, u8 len, u8 data[len] */
	DATA_XMIT_STREAM	= 0x0a, /* u8 id, u8 c */
	RESP_RECV_BLOCK		= 0x0b, /* u8 id, u8 status */
	RESP_RECV_STREAM	= 0x0c, /* u8 id, u8 status */
	CMD_ADDRESS		= 0x0d, /* u8 id */
	CMD_SET_PAN_ID = 0x0f, /* u8 id, u8 u8 panid (MSB first) */
	CMD_SET_SHORT_ADDRESS = 0x10, /* u8 id, u8 u8 address  (MSB first)*/
	CMD_SET_LONG_ADDRESS = 0x11, /* u8 id, u8 u8 u8 u8 u8 u8 u8 u8 address (MSB first) */

	/* Firmware to Driver */
	RESP_OPEN		= 0x81, /* u8 id, u8 status */
	RESP_CLOSE		= 0x82, /* u8 id, u8 status */
	RESP_SET_CHANNEL 	= 0x84, /* u8 id, u8 status */
	RESP_ED			= 0x85, /* u8 id, u8 status, u8 level */
	RESP_CCA		= 0x86, /* u8 id, u8 status */
	RESP_SET_STATE		= 0x87, /* u8 id, u8 status */
	RESP_XMIT_BLOCK		= 0x89, /* u8 id, u8 status */
	RESP_XMIT_STREAM	= 0x8a, /* u8 id, u8 status */
	DATA_RECV_BLOCK		= 0x8b, /* u8 id, u8 lq, u8 len, u8 data[len] */
	DATA_RECV_STREAM	= 0x8c, /* u8 id, u8 c */
	RESP_ADDRESS		= 0x8d, /* u8 id, u8 status, u8 u8 u8 u8 u8 u8 u8 u8 address */
	RESP_SET_PAN_ID     = 0x8f, /* u8 id, u8 status */
	RESP_SET_SHORT_ADDRESS = 0x90, /* u8 id, u8 status */
	RESP_SET_LONG_ADDRESS = 0x91, /* u8 id, u8 status */
};

enum {
	STATE_WAIT_START1,
	STATE_WAIT_START2,
	STATE_WAIT_COMMAND,
	STATE_WAIT_PARAM1,
	STATE_WAIT_PARAM2,
	STATE_WAIT_DATA
};

#endif
