#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

class SerialPC {
private:
    int fd;
    bool connected;
public:
    SerialPC() : fd(-1), connected(false) {}
    ~SerialPC() { close(); }

    int setup(char* portName, int baudrate) {
        fd = ::open(portName, O_RDWR | O_NOCTTY);
        if (fd < 0) {
            fprintf(stderr, "Error opening %s: %s\n", portName, strerror(errno));
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd, &tty) != 0) {
            fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
            return -1;
        }

        speed_t baud;
        switch (baudrate) {
            case 9600:    baud = B9600;   break;
            case 19200:   baud = B19200;  break;
            case 38400:   baud = B38400;  break;
            case 57600:   baud = B57600;  break;
            case 115200:  baud = B115200; break;
            case 230400:  baud = B230400; break;
            case 460800:  baud = B460800; break;
            case 921600:  baud = B921600; break;
            default:      baud = B115200; break;
        }
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);

        tty.c_cflag = baud | CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_cc[VMIN]  = 0;

        tcflush(fd, TCIFLUSH);
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
            return -1;
        }

        connected = true;
        printf("Opened %s @ %d baud\n", portName, baudrate);
        return 0;
    }

    void resetESP32(int wait_sec = 8) {
        if (fd < 0) return;
        int flags;
        ioctl(fd, TIOCMGET, &flags);

        // Pull DTR+RTS low → ESP32 EN pin goes low → reset
        flags |= TIOCM_DTR;
        flags |= TIOCM_RTS;
        ioctl(fd, TIOCMSET, &flags);
        usleep(100000);  // hold 100ms

        // Release → ESP32 boots
        flags &= ~TIOCM_DTR;
        flags &= ~TIOCM_RTS;
        ioctl(fd, TIOCMSET, &flags);

        printf("ESP32 reset. Waiting %ds for setup() (motors entering control mode) ...\n", wait_sec);
        sleep(wait_sec);
        tcflush(fd, TCIFLUSH);  // flush stale data
        printf("Ready.\n");
    }

    int getFd() { return fd; }

    int write(std::string data) {
        if (!connected || fd < 0) return -1;
        return ::write(fd, data.c_str(), data.length());
    }

    int read(char* buffer, unsigned int buf_size) {
        if (!connected || fd < 0) return -1;
        return ::read(fd, buffer, buf_size);
    }

    int close() {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
            connected = false;
        }
        return 0;
    }
};
