/*
 * serial_comm.c
 *
 *  Created on: May 12, 2015
 *      Author: fogatron
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// IPC
/* For Ach IPC */
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>

#include <ach.h>
#include <channellocalizer.h>

//IPC
/* Ach Channel IDs */
ach_channel_t chan_localizer;    // localizer

int uart0_filestream = -1;
int choice=0;
unsigned char tx_buffer[256];
unsigned char *p_tx_buffer;
unsigned char rx_buffer[256];

void transmit_bytes();
void read_bytes();
void ftoa(float,int);

int main(){

	uart0_filestream = open("dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart0_filestream == -1){
		printf("Error - Unable to open UART\n");
	}

	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	while(1){
		read_bytes();
	}
}

void transmit_bytes(){
	if (uart0_filestream !=-1){
		int count = write(uart0_filestream, tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));
		if (count<0){
			printf("UART TX error\n");
		}
	}
}

void read_bytes(){
	if (uart0_filestream != -1){
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
		if(rx_length < 0){
			//error occurred---no bytes to read
		}
		if (rx_length == 0){
			//no data waiting
		}
		else {
			//bytes received
			rx_buffer[rx_length] = '\0';
			printf("%i bytes read : %s\n", rx_length, rx_buffer);
		}
		if (rx_buffer=='map'){
			FILE *fp;
			fp = fopen("~/Documents/map.jpg","r");
			size_t br;
			p_tx_buffer = &tx_buffer[0];
			int buf[256];
			int i;
			while((br=fread(buf,1,255,fp))!=0){
				for(i=0;i<br;i++){
					*p_tx_buffer++ = buf[i];
					if(p_tx_buffer==255){
						transmit_bytes();
						p_tx_buffer = &tx_buffer[0];
					}
				}
			}
			transmit_bytes();
		}
		if (rx_buffer=='meta'){
			FILE *fp;
			fp = fopen("~/Documents/meta.txt","r");
			size_t br;
			p_tx_buffer = &tx_buffer[0];
			int buf[256];
			int i;
			while((br=fread(buf,1,255,fp))!=0){
				for(i=0;i<br;i++){
					*p_tx_buffer++ = buf[i];
					if(p_tx_buffer==255){
						transmit_bytes();
						p_tx_buffer = &tx_buffer[0];
					}
				}
			}
			transmit_bytes();
		}
		if (rx_buffer=='local'){
			//IPC
			//int localizerOpen = ach_open(&chan_localizer, CHAN_LOCALIZER_NAME, NULL);
			//assert(ACH_OK == localizerOpen);

			/*Create initial structure to read from*/
			struct localizer L_localizer;
			memset(&L_localizer, 0, sizeof(L_localizer));

			/*for size check*/
			size_t fs;

			while(1){

				// IPC
				/*Get the current localization values*/
				//localizerOpen = ach_get(&chan_localizer, &L_localizer, sizeof(L_localizer), &fs, NULL, ACH_O_LAST);
				/*if(ACH_OK != localizerOpen){
					assert(sizeof(L_localizer) == fs);
				}*/
				//uint32_t valid = L_localizer.valid;
				/*
				double x = L_localizer.x;
				double y = L_localizer.y;
				uint32_t z = L_localizer.z;
				double THETA = L_localizer.THETA;
				*/
				double x=10.2;
				double y=1.2;
				uint32_t z = 0.1;
				double THETA = 4.3;
				int i;
				for(i=0;i<4;i++){
					if (i==0){
						p_tx_buffer = &tx_buffer[0];
						snprintf(p_tx_buffer,sizeof(tx_buffer),"%f",x);
						int length = (int)floor(log10((float)x)) + 1;
						p_tx_buffer = p_tx_buffer + length;
						transmit_bytes();
					}
					if (i==1){
						p_tx_buffer = &tx_buffer[0];
						snprintf(p_tx_buffer,sizeof(tx_buffer),"%f",y);
						int length = (int)floor(log10((float)y)) + 1;
						p_tx_buffer = p_tx_buffer + length;
						transmit_bytes();
					}
					if (i==2){
						p_tx_buffer = &tx_buffer[0];
						snprintf(p_tx_buffer,sizeof(tx_buffer),"%f",z);
						int length = (int)floor(log10((float)z)) + 1;
						p_tx_buffer = p_tx_buffer + length;
						transmit_bytes();
					}
					if (i==3){
						p_tx_buffer = &tx_buffer[0];
						snprintf(p_tx_buffer,sizeof(tx_buffer),"%f",THETA);
						int length = (int)floor(log10((float)THETA)) + 1;
						p_tx_buffer = p_tx_buffer + length;
						transmit_bytes();
					}
				}
			}
		}
	}
}
