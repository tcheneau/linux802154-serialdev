#include "linux.h"
#define start_command() ((sb[0] == START_BYTE1) && (sb[1] == START_BYTE2))

#include <mc1322x.h>
#include <board.h>
#include <stdio.h> /* For printf() */
#include <errno.h>
#include <string.h>

#ifndef MAC_ADDR_NVM
#define MAC_ADDR_NVM 0x1E000
#endif

#ifndef IEEE802154_ADDR_LEN
#define IEEE802154_ADDR_LEN 8
#endif

#ifndef IEEE802154_SHORT_ADDR_LEN
#define IEEE802154_SHORT_ADDR_LEN 2
#endif


#ifndef BLOCKING_TX
#define BLOCKING_TX 1
#ifndef AUTO_ACK
#define AUTO_ACK 1
#endif
#endif


#ifdef BLOCKING_TX
/* status codes */
#define SUCCESS          0 
#define NO_ACK           5
static volatile uint8_t tx_complete;
static volatile uint8_t tx_status;
#endif

#define ACK_RETRY 5

static volatile uint32_t panid; 
static volatile uint32_t shortaddr;
static volatile uint32_t longaddr_hi;
static volatile uint32_t longaddr_lo;

/* Issues */
/* handle state=TX_STATE, tx_head != 0 in wait for start1 condition better */
/* we can get into getc when there isn't a pending character */
/* maybe this should timeout / protect all the getc's with a uart1_can_get */

void give_to_linux(volatile packet_t *p) {
	uint8_t i;
        /* linux doesn't need these null to frame, but the python tests do */
	uart1_putc(0);
	uart1_putc(0);
	/* send data_recv_block */
	printf("zb");
	uart1_putc(DATA_RECV_BLOCK);
	uart1_putc(p->lqi);
	uart1_putc(p->length); 

	for(i=0; i < p->length ; i++) {
		uart1_putc(p->data[ i + p->offset]);
	}

}

/* How long to wait for command bytes */
#define GETC_TIMEOUT  8192
/* linux uses channels 1-16 the maca driver expects 0-15 */
/* and 802.15.4 is 11-26 (for 2.4GHz) */
#define PHY_CHANNEL_OFFSET 1 

static uint8_t current_channel = 0;

static packet_t cached_p;

void maca_tx_callback(volatile packet_t *p __attribute__((unused))) {
#if BLOCKING_TX
	tx_complete = 1;
	tx_status = p->status;
#endif
}

void set_maca_vars(void) {
	*MACA_MACPANID = panid;
	*MACA_MAC16ADDR = shortaddr;
	*MACA_MAC64HI = longaddr_hi;
	*MACA_MAC64LO = longaddr_lo;

#if AUTO_ACK
	*MACA_TXACKDELAY = 68; /* 68 puts the tx ack at about the correct spot */
	*MACA_RXACKDELAY = 30; /* start reception 100us before ack should arrive */
	*MACA_RXEND = 180; /* 750us receive window */
#endif /* AUTO_ACK */
}

int timed_getc(volatile uint8_t *c) {
	volatile uint32_t timeout;
	for(timeout = 0; timeout < GETC_TIMEOUT; timeout++) {
		if(uart1_can_get()) {
			*c = uart1_getc();
			return 1;
		} 
	}
	return -ETIMEDOUT;
}

void main(void) {	
	volatile uint8_t sb[NUM_START_BYTES];
	volatile uint32_t i;
	volatile uint8_t cmd, parm1;
	static volatile uint8_t state = IDLE_MODE;
	volatile packet_t *p = 0;

	trim_xtal();
	uart_init(UART1, 921600);
	maca_init();
	maca_off();

	while(1) {

		/* clear out sb */
		for(i = 0; i < NUM_START_BYTES; i++) {
			sb[i] = 0;
		}

		/* recieve bytes until we see the first start byte */
		/* this syncs up to the commands */
		while(sb[0] != START_BYTE1) {

			check_maca();
			set_maca_vars();

			if((state == TX_MODE) &&
			   (tx_head == 0)) {
				/* this could happen if the RX_MODE */
				/* set_state command is missed */
				state = RX_MODE;
			} 
			if(state == RX_MODE) {
				if ( rx_head && (p = rx_packet()) ) {
					give_to_linux(p);
					free_packet(p);
					continue;
				}
			}
			if(uart1_can_get()) { sb[0] = uart1_getc(); }
		}

		/* receive start bytes */
		for(i=1; i<NUM_START_BYTES; i++) {
			if(timed_getc(&sb[i]) < 0) {
				/* timedout without bytes */
				/* invalidate the start command */
				sb[0] = 'X'; sb[1] = 'X';
			}
		}
		
		if(start_command()) {
			/* do a command */
			cmd = 0;

			if(timed_getc(&cmd) < 0 ) {
				cmd = 0;
			}
			
			switch(cmd)
			{
			case CMD_OPEN:
				set_power(0x12); /* 4.5dbm */
				set_channel(15); /* channel 26 */
				maca_on();

				set_maca_vars();
#if AUTO_ACK
				set_prm_mode(AUTOACK);
#endif /* AUTO_ACK */
				printf("zb");
				uart1_putc(RESP_OPEN);
				uart1_putc(STATUS_SUCCESS);
				break;
			case CMD_CLOSE:
//				maca_off();
				free_all_packets();
				printf("zb");
				uart1_putc(RESP_CLOSE);
				uart1_putc(STATUS_SUCCESS);
				break;
			case CMD_SET_CHANNEL:
				maca_off();				
				if(timed_getc(&parm1) < 0 ) {
					printf("zb");
					uart1_putc(RESP_SET_CHANNEL);
					uart1_putc(STATUS_ERR);
					break;
				}
				current_channel = parm1;
				set_channel(parm1-PHY_CHANNEL_OFFSET);
				maca_on();
				printf("zb");
				uart1_putc(RESP_SET_CHANNEL);
				uart1_putc(STATUS_SUCCESS);
				break;
			case CMD_ED:
				printf("zb");
				uart1_putc(RESP_ED);
				uart1_putc(STATUS_ERR);
				uart1_putc(0);
				break;
			// legacy command: should never be called with the new linux kernel driver (serial.c)
			case CMD_CCA:
				printf("zb");
				uart1_putc(RESP_CCA);
				uart1_putc(STATUS_SUCCESS);
				break;
			// legacy command: should never be called with the new linux kernel driver (serial.c)
			case CMD_SET_STATE:
				if(timed_getc(&state) < 0 ) {
					printf("zb");
					uart1_putc(RESP_SET_STATE);
					uart1_putc(STATUS_ERR);
					state = RX_MODE;
					break;
				}
				printf("zb");
				uart1_putc(RESP_SET_STATE);
				uart1_putc(STATUS_SUCCESS);
				break;
			case DATA_XMIT_BLOCK: {
#if AUTO_ACK
				volatile uint8_t retry = 0;
#endif
				//memset(&cached_p, 0, sizeof(cached_p));


				if(timed_getc(& cached_p.length) < 0 ) {
					printf("zb");
					uart1_putc(RESP_XMIT_BLOCK);
					uart1_putc(STATUS_ERR);
					state = RX_MODE;
					break;
				}
				
				for(i=0; i < cached_p.length; i++) {
					if(timed_getc(&(cached_p.data[i])) < 0 ) {
						printf("zb");
						uart1_putc(RESP_XMIT_BLOCK);
						uart1_putc(STATUS_ERR);
						state = RX_MODE;
						goto data_xmit_block_end;
					}
				}

				state = TX_MODE;

sendpkt:

				/* send packet here */
				if( ( p = get_free_packet() ) ) {
#if BLOCKING_TX
					tx_complete = 0;
#endif /* BLOCKING_TX */
					int status = STATUS_SUCCESS;

					memcpy((packet_t * ) p, &cached_p, sizeof(packet_t));

					tx_packet(p);
					
#if BLOCKING_TX
					/* block until tx_complete, set by maca_tx_callback */
					while(!tx_complete && (tx_head != 0)) {;}

					switch(tx_status) {
						case SUCCESS:
							status = STATUS_SUCCESS;
							break;
#if AUTO_ACK
						case NO_ACK:
							if (retry < ACK_RETRY)
							{
								++retry;
								goto sendpkt;
							}
							else
							{
								state = RX_MODE;
								status = STATUS_BUSY_TX;
							}
							break;
#endif /* AUTO_ACK */
						case BUSY: // CCA failed
							// TODO: implement simplistic CSMA-CA
							// for now, just try to send as fast as possible
							goto sendpkt;
							break;
	
						default:
							status = STATUS_ERR;
					}
#endif /* BLOCKING_TX */
					printf("zb");
					uart1_putc(RESP_XMIT_BLOCK);
					uart1_putc(status);

				} else {
					printf("zb");
					uart1_putc(RESP_XMIT_BLOCK);
					uart1_putc(STATUS_BUSY);
				}
				state = RX_MODE;
data_xmit_block_end:
				break;
			}
			case CMD_ADDRESS: {
				uint8_t buf[IEEE802154_ADDR_LEN];
				nvmType_t type = 0;
				nvmErr_t err;	
				int i;

				printf("zb");
				uart1_putc(RESP_ADDRESS);

				vreg_init();
				err = nvm_detect(gNvmInternalInterface_c, &type);
				err = nvm_read(gNvmInternalInterface_c, type, buf, MAC_ADDR_NVM, IEEE802154_ADDR_LEN);

				uart1_putc(STATUS_SUCCESS);

				for (i = 0; i < IEEE802154_ADDR_LEN; i++)
					uart1_putc(buf[i]);
				break;
			}
			case CMD_SET_PAN_ID: {
				uint8_t buf[2];
				for (i=0; i < 2; i++)
				{
						if(timed_getc(&(buf[i])) < 0 ) {
							printf("zb");
							uart1_putc(RESP_SET_PAN_ID);
							uart1_putc(STATUS_ERR);
							break;
						}
				}

				panid = (buf[0] << 8) | buf[1];
				*MACA_MACPANID = panid;
				
				printf("zb");
				uart1_putc(RESP_SET_PAN_ID);
				uart1_putc(STATUS_SUCCESS);
				break;
			}
			case CMD_SET_SHORT_ADDRESS: {
				uint8_t buf[IEEE802154_SHORT_ADDR_LEN];
				for (i=0; i < IEEE802154_SHORT_ADDR_LEN; i++)
				{
						if(timed_getc(&(buf[i])) < 0 ) {
							printf("zb");
							uart1_putc(RESP_SET_SHORT_ADDRESS);
							uart1_putc(STATUS_ERR);
							break;
						}
				}

				shortaddr = (buf[0] << 8) | buf[1];
				*MACA_MAC16ADDR = shortaddr;

				printf("zb");
				uart1_putc(RESP_SET_SHORT_ADDRESS);
				uart1_putc(STATUS_SUCCESS);
				break;
			}
			case CMD_SET_LONG_ADDRESS: {
				nvmType_t type = 1; // could be detected, but somehow it fails the first time for me
				nvmErr_t err;	
				uint8_t buf[IEEE802154_ADDR_LEN];

				for (i=0; i < IEEE802154_ADDR_LEN; i++)
				{
						if(timed_getc(&(buf[i])) < 0 ) {
							printf("zb");
							uart1_putc(RESP_SET_LONG_ADDRESS);
							uart1_putc(STATUS_ERR);
							break;
						}
				}

				printf("zb");
				uart1_putc(RESP_SET_LONG_ADDRESS);
				/* store in RAM */
				longaddr_hi = (buf[0] << 24) |(buf[1] << 16) |(buf[2] << 8) | buf[3];
				longaddr_lo = (buf[4] << 24) |(buf[5] << 16) |(buf[6] << 8) | buf[7];
				*MACA_MAC64HI = longaddr_hi;
				*MACA_MAC64LO = longaddr_lo;

				/* store to flash */
				vreg_init();

				//	somehow, detection fails the first time, so it can not be relied on
				//	nvm_detect(gNvmInternalInterface_c, &type);

				/* disable flash protection */
				nvm_setsvar(0);

				nvm_erase(gNvmInternalInterface_c, type, 0x40000000); 

				err =  nvm_write(gNvmInternalInterface_c, type, buf, MAC_ADDR_NVM, IEEE802154_ADDR_LEN); 
				if (err)
					uart1_putc(STATUS_ERR);
				else
					uart1_putc(STATUS_SUCCESS);

				/* TC: I'm not sure it is useful */
				for(i=0; i< IEEE802154_ADDR_LEN; ++i)
					buf[i] = 0;

				nvm_read(gNvmInternalInterface_c, type, buf, MAC_ADDR_NVM, IEEE802154_ADDR_LEN);
				
				/* enable flash protection */
				nvm_setsvar(1);

				break;
			}
			default:
				break;
			}
		}
	}
}
