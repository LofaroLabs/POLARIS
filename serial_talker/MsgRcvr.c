/**
 * A simple C program that connects to the serial port and receivings
 * messages from the Arduino.
 *
 * The message format is:
 * [PROTEUS_BEGIN] [unsigned int 1] [unsigned int 2] [unsigned int 3] [CHECKSUM]
 *
 * Where:
 *  [PROTEUS_BEGIN] = the message begin key (1 byte)
 *  [unsigned int 1-3] = the message body which contain three unsigned integer values (4 bytes each)
 *  [CHECKSUM] = the sum of the bytes in the message body (1 byte)
 *
 * The unsigned int values are formatted using little endian.
 *
 * @author Chien-Liang Fok
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <poll.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define PROTEUS_BEGIN 0x24
#define MSG_LENGTH 14 // 1 byte header, three 4-byte longs, 1 byte checksum
#define SERIAL_BUFFER_SIZE 100
#define CYCLE_TIME_NS 100000000 //10hz

/**
 * Defines the message.
 */
struct ThreeLongMsg {
	unsigned int val1;
	unsigned int val2;
	unsigned int val3;
};

void processMsg(char* msg) {
	printf("Processing msg...\n");

	int i;
	printf(" - Raw bytes: ");
	for (i=0; i < MSG_LENGTH; i++) {
		printf("0x%.2x ", msg[i] & 0xff);
	}
	printf("\n");
	
	// Compute the checksum, which is the sum of all the bytes in the message body
	char sum = 0;
	for (i=1; i < MSG_LENGTH-1; i++) {
		sum += (msg[i] & 0xff);
		sum &= 0xff; 
	}
	if (sum == msg[MSG_LENGTH-1])
		printf(" - Checkum: Passed!\n");
	else
		printf(" - Checksum: Failed! Expected 0x%.2x, calculated 0x%.2x\n", msg[MSG_LENGTH-1] & 0xff, sum & 0xff);
	
	// Extract the values within the message
	struct ThreeLongMsg* lmsg = (struct ThreeLongMsg*)(&msg[1]);
	printf(" - Message: val1 = %u, val2 = %u, val3 = %u\n", lmsg->val1, lmsg->val2, lmsg->val3);

	//long val1 = (msg[1] << 24) + (msg[2] << 16) + (msg[3] << 8) + msg[4];
	//long val2 = (msg[5] << 24) + (msg[6] << 16) + (msg[7] << 8) + msg[8];
	//long val3 = (msg[9] << 24) + (msg[10] << 16) + (msg[11] << 8) + msg[12];
	//printf(" - Message: val1 = %lu, val2 = %lu, val3 = %lu\n", val1, val2, val3);
}

int main() {
	printf("Opening serial port %s...\n", SERIAL_PORT);
	
	/*
	 * Open the serial port.  Use non-blocking at first, in case there's no micro-controller attached.
	 *
	 * For documentation on the open command, see: 
	 * http://pubs.opengroup.org/onlinepubs/000095399/functions/open.html
	 */
	int fd = open(SERIAL_PORT, O_RDWR | O_NONBLOCK | O_FSYNC, S_IRUSR | S_IWUSR);
	
	// Error condition check
	if (fd == -1) {
		printf("ERROR: Unable to open serial port: %s (%d)\n", strerror(errno), errno);
		return -1;
	} else {
		printf("Serial port opened...\n");
	}
	
	/*
	 * Flushes data that was received over the serial port but not yet read.
	 * See:  http://opengroup.org/onlinepubs/007908775/xsh/tcflush.html
	 */
	if(tcflush(fd, TCIFLUSH) < 0 ) {
		printf("ERROR: Unable to flush serial port.\n");
		close(fd);
		return -1;
	}
	
	/*
	 * Get the attribues associated with the serial port
	 * See: http://www.opengroup.org/onlinepubs/009695399/functions/tcgetattr.html
	 */
	struct termios term;
	if(tcgetattr(fd, &term) < 0 ) {
		printf("ERROR: Unable to get serial port attributes.\n");
		close(fd);
		return -1;
	}
	
	/*
	 * This function provides an easy way to set up *termios-p for what has traditionally been 
	 * called “raw mode” in BSD. This uses noncanonical input, and turns off most processing 
	 * to give an unmodified channel to the terminal.
	 *
	 * See: http://linux.die.net/man/3/cfmakeraw
	 * And: http://www.gnu.org/s/libc/manual/html_node/Noncanonical-Input.html 
	 */
	cfmakeraw(&term); 
	
	// Set 115200 baud
	// B57600 57600 baud
	cfsetispeed(&term, B115200); 
	cfsetospeed(&term, B115200);
	
	/*
	 * Apply the changes to the serial port.
	 * See: http://www.delorie.com/gnu/docs/glibc/libc_360.html
	 *
	 * TCSAFLUSH tells the underlying OS to wait until the all queued output has been written,
	 * and to discard any queued input
	 */
	if (tcsetattr(fd, TCSAFLUSH, &term) < 0 ) {
		printf("ERROR: Unable to set serial port attributes.\n");
		close(fd);
		return -1;
	}
	
	char msg[MSG_LENGTH];
	int msgIndx = 0;
	bool rcvMsg = false;
	
	while (1) {
		struct pollfd ufd[1];
		int timeout = -1;
		
	
		if (fd < 0) {
			printf("ERROR: fd < 0\n");
			return -1;
		}
	
		ufd[0].fd = fd;
		ufd[0].events = POLLIN; // data may be read without waiting
	
		int retval = poll(ufd,1,timeout); // see if there is data ready to be read from the serial port
	
		if(retval < 0) {
			if(errno == EINTR) {
				// The call was interrupted, ignore the error, we will try again later...
			} else if(errno == EINTR) {
				// Allocation of internal data structures failed, but the request may be attempted again.
				// Ignore the error, we will try again later...
			} else {
				printf("ERROR: Problem w/ poll, errno = %i\n", errno);
				return -1;
			}
		} else if(retval == 0) {
			printf("ERROR: poll timeout\n");
			return -1;
		} else {
			if (ufd[0].revents & POLLIN) {  // if serial port data is ready to be received
				int numread;
				char databuf[SERIAL_BUFFER_SIZE]; // for temporarily holding bytes from the serial port
				if((numread = read(fd, databuf, SERIAL_BUFFER_SIZE)) < 0) {
					printf("ERROR: problem while reading from serial port!\n");
					return -1;
				} else {
					int i;
					for (i = 0; i < numread; i++) {
						if (rcvMsg) {
							msg[msgIndx++] = databuf[i];
							if (msgIndx == MSG_LENGTH) {
								processMsg(msg);
								rcvMsg = false;
							}
						} else {
							if (databuf[i] == PROTEUS_BEGIN) {
								msg[0] = PROTEUS_BEGIN;
								msgIndx = 1;
								rcvMsg = true;
							}						
						}
					}
					printf("\n");
				}
			} else {
				printf("No data.\n");
			}
		}
	
		struct timespec ts;
		ts.tv_sec = 0;
		ts.tv_nsec = CYCLE_TIME_NS;
		nanosleep(&ts, NULL);
	} 

	// Close the serial port.
	close(fd);
	
	// Terminate program without error.
	return 0;
}
