#include <stdio.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

void error_message(const char *msg, int data) {
	printf(msg, data);
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof (tty));
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

int main(int argc, char *argv[]) {
	puts("opening port");
	int f = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_SYNC);
	if (f >= 0) {
		puts("opened OK");
	}
	else {
		puts("problems");
	}

	set_interface_attribs(f, B2400, 0);
	ssize_t cnt;
	char buff[10];

	write(f, "\xff\x01\x00\x02\x00\x00\x03", 7);
	//cnt = read(f, buff, 4); 
	//write(f, "\xff\x01\x00\x04\x01\x01\x07", 7);
	usleep(20000);
	write(f, "\xff\x01\x00\x00\x00\x00\x01", 7);
	puts("sent...");
	cnt = read(f, buff, 4); 
	printf("received %d bytes", cnt);
	close(f);
	return 0;
}
