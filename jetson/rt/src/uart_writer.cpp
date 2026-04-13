#include "uart_writer.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

UartWriter::UartWriter(const char* device) {
    fd_ = open(device, O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        fprintf(stderr, "[UartWriter] Cannot open %s: %s — entering console mode\n",
                device, strerror(errno));
        console_mode_ = true;
        return;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    tcgetattr(fd_, &tty);

    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    tty.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[UartWriter] tcsetattr: %s — entering console mode\n",
                strerror(errno));
        close(fd_);
        fd_ = -1;
        console_mode_ = true;
        return;
    }

    // Wait for ESP32 setup() to complete (enter_control_mode for all motors)
    printf("[UartWriter] Opened %s @ 921600 baud — waiting 5s for ESP32 boot...\n", device);
    sleep(5);
    tcflush(fd_, TCIOFLUSH);
    printf("[UartWriter] ESP32 ready.\n");
}

UartWriter::~UartWriter() {
    if (fd_ >= 0) close(fd_);
}

void UartWriter::resetESP32(int wait_sec) {
    if (fd_ < 0 || console_mode_) return;

    printf("[UartWriter] Waiting %ds for ESP32 setup()...\n", wait_sec);
    sleep(wait_sec);
    tcflush(fd_, TCIOFLUSH);
    printf("[UartWriter] Ready.\n");
}

bool UartWriter::send_all(float m1, float m2, float m3,
                          float /*kp*/, float /*kd*/)
{
    if (console_mode_) return true;

    // Text protocol — exact same format as serial_write_threejoints_wrist_testing3
    // which made all 3 motors run. One motor per write:
    //   write("id\n")  → ESP32 reads id via readStringUntil('\n')
    //   write("theta_millirad\n") → ESP32 reads theta

    struct { int id; int theta; } cmds[3] = {
        { cfg::MOTOR_IDS[1], (int)(m2 * 1000) },
        { cfg::MOTOR_IDS[2], (int)(m3 * 1000) },
        { cfg::MOTOR_IDS[0], (int)(m1 * 1000) },
    };

    for (int i = 0; i < 3; i++) {
        char id_buf[16], th_buf[16];
        int id_len = snprintf(id_buf, sizeof(id_buf), "%d\n", cmds[i].id);
        int th_len = snprintf(th_buf, sizeof(th_buf), "%d\n", cmds[i].theta);

        ::write(fd_, id_buf, id_len);
        tcdrain(fd_);
        ::write(fd_, th_buf, th_len);
        tcdrain(fd_);
        usleep(500);
    }
    return true;
}
