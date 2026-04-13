#pragma once

#include "motor_command.h"

class UartWriter {
public:
    explicit UartWriter(const char* device);
    ~UartWriter();

    UartWriter(const UartWriter&) = delete;
    UartWriter& operator=(const UartWriter&) = delete;

    /* DTR/RTS reset, blocking wait for ESP32 setup, then switch to O_NDELAY. */
    void resetESP32(int wait_sec = 10);

    /* Fire-and-forget: write 3 packed MotorCommands (45 bytes total). */
    bool send_all(float m1, float m2, float m3,
                  float kp, float kd);

    int fd() const { return fd_; }
    bool is_console_mode() const { return console_mode_; }

private:
    int  fd_ = -1;
    bool console_mode_ = false;
};
