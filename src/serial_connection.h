#ifndef SERIAL_CONN_H
#define SERIAL_CONN_H

#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include "custom_defines.h"

#define SERIAL_BUFFSIZE 64

extern char serial_buffer[SERIAL_BUFFSIZE];

void init_serial_port(int &tty_fd) { // 115200 baud 8n1 blocking read
    struct termios tty_opt;
    memset(&tty_opt, 0, sizeof(tty_opt));
    tty_opt.c_cflag = CS8 | CLOCAL | CREAD; // CS8: 8n1, CLOCAL: local connection, no modem contol, CREAD: enable receiving characters
    tty_opt.c_iflag = 0;
    tty_opt.c_oflag = 0;
    tty_opt.c_lflag = 0;     // non-canonical mode
    tty_opt.c_cc[VMIN] = 1;  // blocking read until 1 character arrives
    tty_opt.c_cc[VTIME] = 0; // inter-character timer unused
    cfsetospeed(&tty_opt, B115200);
    cfsetispeed(&tty_opt, B115200);
    tcsetattr(tty_fd, TCSANOW, &tty_opt);
}

void read_data(int &tty_fd, char *buffer, int &size) {
    int err=0, i=0;
    char temp=0;

    while(temp != '\n') {
        err = read(tty_fd, &temp, 1);
        if(err < 0) {
            LOG_ERROR_C("Error reading from serial port: " << strerror(errno));
            continue;
        }
        buffer[i] = temp;
        i++;
    }

}



#endif // SERIAL_CONN_H