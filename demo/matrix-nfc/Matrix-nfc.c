#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <linux/reboot.h>
#include "libfahw.h"

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

void set_mincount(int fd, int mcount)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error tcgetattr: %s\n", strerror(errno));
        return;
    }

    tty.c_cc[VMIN] = mcount ? 1 : 0;
    tty.c_cc[VTIME] = 5;        /* half second timer */

    if (tcsetattr(fd, TCSANOW, &tty) < 0)
        printf("Error tcsetattr: %s\n", strerror(errno));
}


const unsigned char wake[24]={
  0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x03, 0xfd, 0xd4, 0x14, 0x01, 0x17, 0x00};//wake up NFC module

const unsigned char tag[11]={
  0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4, 0x4A, 0x01, 0x00, 0xE1, 0x00};//detecting tag command

void wake_card(int fd) {
    int wlen;
    wlen = write(fd, wake, sizeof(wake));
    if (wlen != sizeof(wake)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */
}

void send_tag(int fd) {
    int wlen;
    wlen = write(fd, tag, sizeof(tag));
    if (wlen != sizeof(tag)) {
        printf("Error from write: %d, %d\n", wlen, errno);
    }
    tcdrain(fd);    /* delay for output */
}

int read_ACK(int fd, int size) {
    unsigned char buf[size];
    int rdlen = -1;
    int len = -1;

    fd_set set;
    struct timeval timeout;
    int rv;

    FD_ZERO(&set);
    FD_SET(fd, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 3000;

    rv = select(fd + 1, &set, NULL, NULL, &timeout);
    if (rv == -1)
        perror("select");
    else if(rv == 0)
        printf("timeout");
    else
        rdlen = read(fd, buf, sizeof(buf));

    // rdlen = read(fd, buf, sizeof(buf));
    len = rdlen;
    if (rdlen > 0) {
        unsigned char   *p;
        printf("Read %d:", rdlen);
        for (p = buf; rdlen-- > 0; p++)
            printf(" 0x%x", *p);
        printf("\n");
    } else if (rdlen < 0) {
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
    }

    return len;
}

int countup = 0;
int state = 0;
int pin = GPIO_PIN(13);
void lightOn() {
    int ret = -1;
    int value = GPIO_HIGH;
    if (state == 0) {
        if ((ret = setGPIOValue(pin, value)) > 0) {
            printf("GPIO_PIN(%d) value is %d\n", pin, value);
        } else {
            printf("setGPIOValue(%d) failed\n", pin);
        }
        state = 1;
    }
}

void lightOff() {
    int ret = -1;
    int value = GPIO_LOW;
    if (state == 1) {
        if ((ret = setGPIOValue(pin, value)) > 0) {
            printf("GPIO_PIN(%d) value is %d\n", pin, value);
        } else {
            printf("setGPIOValue(%d) failed\n", pin);
        }
        state = 0;
    }
}

int main()
{
    char *portname = "/dev/ttyAMA2";
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }
    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);
    //set_mincount(fd, 0);                /* set to pure timed read */

    int board;
    int ret = -1;

    if ((board = boardInit()) < 0) {
        printf("Fail to init board\n");
        return -1;
    } 

    if ((ret = exportGPIOPin(pin)) == -1) {   
        printf("exportGPIOPin(%d) failed\n", pin);
    }
    if ((ret = setGPIODirection(pin, GPIO_OUT)) == -1) {
        printf("setGPIODirection(%d) failed\n", pin);
    }

    wake_card(fd);
    usleep(100000);
    read_ACK(fd, 15);
    usleep(100000);

    int rdlen = -1;
    /* simple noncanonical input */
    do {
        send_tag(fd);
        usleep(100000);
        rdlen = read_ACK(fd, 25);
        if (rdlen > 6) {
            lightOn();
        } else {
            lightOff();
            countup = 0;
        }
        printf("countup: %d", countup);
        if (countup > 100) {
            system("/sbin/shutdown -P now");
        } else {
            countup++;
        }
        usleep(100000);
    } while (1);

    unexportGPIOPin(pin);
}