#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <sys/types.h>

// see http://www.cmrr.umn.edu/~strupp/serial.html

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
//int serial_init(const char* serialport, int baud) {
//	struct termios toptions;
//	int fd;
//
////	fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
//	fd = open(serialport, O_RDWR | O_NOCTTY | O_SYNC);
//	//fd = open(serialport, O_RDWR | O_NONBLOCK);
//	//fd = open(serialport, O_RDWR);
//
//	if (fd == -1) {
//		perror("serialport_init: Unable to open port ");
//		return -1;
//	}
//
//	//int iflags = TIOCM_DTR;
//	//ioctl(fd, TIOCMBIS, &iflags);     // turn on DTR
//	//ioctl(fd, TIOCMBIC, &iflags);    // turn off DTR
//
//	if (tcgetattr(fd, &toptions) < 0) {
//		perror("serialport_init: Couldn't get term attributes");
//		return -1;
//	}
//	speed_t brate = baud; // let you override switch below if needed
//	switch (baud) {
//	case 4800:
//		brate = B4800;
//		break;
//	case 9600:
//		brate = B9600;
//		break;
//#ifdef B14400
//		case 14400: brate=B14400; break;
//#endif
//	case 19200:
//		brate = B19200;
//		break;
//#ifdef B28800
//		case 28800: brate=B28800; break;
//#endif
//	case 38400:
//		brate = B38400;
//		break;
//	case 57600:
//		brate = B57600;
//		break;
//	case 115200:
//		brate = B115200;
//		break;
//	case 230400:
//		brate = B230400;
//		break;
//	case 460800:
//		brate = B460800;
//		break;
//	case 576000:
//		brate = B576000;
//		break;
//	case 921600:
//		brate = B921600;
//		break;
//	case 1152000:
//		brate = B1152000;
//		break;
//	case 1500000:
//		brate = B1500000;
//		break;
//	case 2000000:
//		brate = B2000000;
//		break;
//	case 2500000:
//		brate = B2500000;
//		break;
//	case 3000000:
//		brate = B3000000;
//		break;
//	case 3500000:
//		brate = B3500000;
//		break;
//	case 4000000:
//		brate = B4000000;
//		break;
//	}
//	cfsetispeed(&toptions, brate);
//	cfsetospeed(&toptions, brate);
//
//	// 8N1
//	toptions.c_cflag &= ~PARENB;
//	toptions.c_cflag &= ~CSTOPB;
//	toptions.c_cflag &= ~CSIZE;
//	toptions.c_cflag |= CS8;
//	// no flow control
//	toptions.c_cflag &= ~CRTSCTS;
//
//	// disable hang-up-on-close to avoid reset
//	//toptions.c_cflag &= ~HUPCL;
//
//	// turn on READ & ignore ctrl lines
//	toptions.c_cflag |= CREAD | CLOCAL;
//	// turn off s/w flow ctrl
//	toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
//
//	// make raw
//	toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//	toptions.c_oflag &= ~OPOST;
//
//	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
//	toptions.c_cc[VMIN] = 0;
//	toptions.c_cc[VTIME] = 0;
//	//toptions.c_cc[VTIME] = 20;
//
//	tcsetattr(fd, TCSANOW, &toptions);
//	if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0) {
//		perror("init_serialport: Couldn't set term attributes");
//		return -1;
//	}
//
//	return fd;
//}

int set_interface_attribs(int fd, speed_t speed, int parity) {
	// Tersmios general terminal interface provides an interface to asynchronous communications devices
	struct termios tty;
	// Empty the vector
	memset(&tty, 0, sizeof(tty));
	// tcgetattr gets the parameters associated with the object referred by fd
	if (tcgetattr(fd, &tty) != 0) {
		printf("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);
	// Wavy line clear cooresponding setting bit
	tty.c_cflag &= ~PARENB;            // Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;

	tty.c_cflag |= CS8;				   // Data length has 8 bits

	tty.c_cflag &= ~CRTSCTS;           // no flow control
	//tty.c_cc[VMIN] = 1;                  // read doesn't block
	//tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
	tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	cfmakeraw(&tty);

	tcflush( fd, TCIFLUSH );

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking(int fd, int should_block) {
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0) {
		printf("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN] = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
		printf("error %d setting term attributes", errno);
}

int serial_init(const char* serialport, int baud) {

	int fd = open(serialport, O_RDWR | O_NOCTTY);

	speed_t brate = baud; // let you override switch below if needed
	switch (baud) {
	case 4800:
		brate = B4800;
		break;
	case 9600:
		brate = B9600;
		break;
	case 19200:
		brate = B19200;
		break;
	case 38400:
		brate = B38400;
		break;
	case 57600:
		brate = B57600;
		break;
	case 115200:
		brate = B115200;
		break;
	case 230400:
		brate = B230400;
		break;
	case 460800:
		brate = B460800;
		break;
	case 576000:
		brate = B576000;
		break;

	}

	set_interface_attribs(fd, brate, 0);
	//set_blocking(fd, 0);

	return fd;
}

//
int serial_close(int fd) {
	return close(fd);
}

// BUFF is a char value that indicates a certain memory
int serial_recv_byte(int fd, char *buff) {
	// READ fn transmit 1 byte to memory buff points to, return # of bytes successfully read
	return read(fd, buff, 1);
}

// CAUTION: blocking / busy waiting send

int serial_send_n_bytes(int fd, const char* buff, size_t n) {

	int ret = 0;
	int count = 0;

	while (count < n) {
		// return # of bytes successfully written
		ret = write(fd, buff, n);
		if (ret < 0) {
			return ret;
		}
		count += ret;
	}
	return 0;
}

// CAUTION: blocking / busy waiting receive
int serial_recv_n_bytes(int fd, char *buff, size_t n) {

	int ret = 0;
	int count = 0;

	while (count < n) {
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		ret = read(fd, buff + count, n - count);
		if (ret < 0) {
			return ret;
		}
		count += ret;
	}

	return count;
}

//
int serial_flush(int fd) {
	return tcflush(fd, TCIOFLUSH);
}

