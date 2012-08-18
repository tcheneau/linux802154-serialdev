#include "linux.h"
#define start_command() ((sb[0] == START_BYTE1) && (sb[1] == START_BYTE2))

#include <mc1322x.h>
#include <board.h>
#include <stdio.h> /* For printf() */
#include <errno.h>

#ifndef MAC_ADDR_NVM
#define MAC_ADDR_NVM 0x1E000
#endif

#ifndef IEEE802154_ADDR_LEN
#define IEEE802154_ADDR_LEN 8
#endif

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

/* Baud rate */
#define MOD 9999
/*  1843200 bps, INC=6142, MOD=9999, 24Mhz 16x samp */
/*  921600 bps, INC=6142, MOD=9999, 24Mhz 8x samp */
/*  230400 bps, INC=767, MOD=9999, 24Mhz 16x samp */
/*  115200 bps, INC=767, MOD=9999, 24Mhz 8x samp */
#define INC 6142 
/*  921600 bps, MOD=9999, 24Mhz 16x samp */
//#define INC 3071 
#define SAMP UCON_SAMP_8X
//#define SAMP UCON_SAMP_16X

/* How long to wait for command bytes */
#define GETC_TIMEOUT  8192
/* linux uses channels 1-16 the maca driver expects 0-15 */
/* and 802.15.4 is 11-26 (for 2.4GHz) */
#define PHY_CHANNEL_OFFSET 1 


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
	uart1_init(INC, MOD, SAMP);
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
			case CMD_CCA:
				printf("zb");
				uart1_putc(RESP_CCA);
				uart1_putc(STATUS_SUCCESS);
				break;
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
			case DATA_XMIT_BLOCK:

				/* send packet here */
				if( ( p = get_free_packet() ) ) {

					if(timed_getc(&p->length) < 0 ) {
						printf("zb");
						uart1_putc(RESP_XMIT_BLOCK);
						uart1_putc(STATUS_ERR);
						state = RX_MODE;
						free_packet(p);
						break;
					}
					
					for(i=0; i < p->length; i++) {
						if(timed_getc(&(p->data[ i + p->offset])) < 0 ) {
							printf("zb");
							uart1_putc(RESP_XMIT_BLOCK);
							uart1_putc(STATUS_ERR);
							state = RX_MODE;
							free_packet(p);
							goto data_xmit_block_end;
						}
					}
				      					
					tx_packet(p);
					
					printf("zb");
					uart1_putc(RESP_XMIT_BLOCK);
					uart1_putc(STATUS_SUCCESS);

				} else {
					printf("zb");
					uart1_putc(RESP_XMIT_BLOCK);
					uart1_putc(STATUS_BUSY);
				}
data_xmit_block_end:
				break;
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
			default:
				break;
			}
		}
	}
}
