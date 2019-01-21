/*
 * main.c
 *
 *  Created on: Jul 30, 2018
 *      Author: jnevens
 */
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>


// TODO: https://github.com/ricki-z/SDS011/blob/master/SDS011.cpp

static const uint8_t wakeupcmd[] = {
	0x01,	// head
};

static const uint8_t sleepcmd[] = {
	0xAA,	// head
	0xB4,	// command id
	0x06,	// data byte 1
	0x01,	// data byte 2 (set mode)
	0x00,	// data byte 3 (sleep)
	0x00,	// data byte 4
	0x00,	// data byte 5
	0x00,	// data byte 6
	0x00,	// data byte 7
	0x00,	// data byte 8
	0x00,	// data byte 9
	0x00,	// data byte 10
	0x00,	// data byte 11
	0x00,	// data byte 12
	0x00,	// data byte 13
	0xFF,	// data byte 14 (device id byte 1)
	0xFF,	// data byte 15 (device id byte 2)
	0x05,	// checksum
	0xAB	// tail
};

void sds011_sleep(int fd) {
	write(fd, sleepcmd, sizeof(sleepcmd));
}
void sds011_wakeup(int fd) {
	write(fd, wakeupcmd, sizeof(wakeupcmd));
}

int set_interface_attribs(int fd, int speed, int parity)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "error %d from tcgetattr\n", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN] = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
									 // enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block)
{
	struct termios tty;
	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0) {
		fprintf(stderr, "error %d from tggetattr\n", errno);
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		fprintf(stderr, "error %d setting term attributes\n", errno);
}

int main(int argc, char *argv[])
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s: [port]\n", argv[0]);
		return -1;
	}


	int fd = open (argv[1], O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "error %d opening %s: %s\n", errno, argv[1], strerror(errno));
		return -1;
	}

	set_interface_attribs(fd, B9600, 0);
	set_blocking(fd, 1);

	sds011_wakeup(fd);

	for(int i=0;1;i++) {
		uint8_t buf[10];
		int n = read(fd, buf, sizeof buf);
		if (n == 10) {
			if (i == 10) {
				float ppm_25 = ((buf[3] << 8) + buf[2]) / 10.0f;
				float ppm_10 = ((buf[5] << 8) + buf[4]) / 10.0f;

				/*
				printf("read [%d]:", n);
				for (int i =0; i < 10; i++) {
					printf(" %02x", buf[i]);
				}
				*/

				printf(" PM2.5:\t%f\n", ppm_25);
				printf(" PM10:\t%f\n", ppm_10);
				break;
			}
		}
	}

	sds011_sleep(fd);

	close(fd);

	return 0;
}
