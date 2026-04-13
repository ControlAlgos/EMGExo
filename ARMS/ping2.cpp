#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdlib.h>
#include <termios.h>
#include "SerialLinux.h"

int main(int argc, char* argv[]) {
    int target_id = 2;
    if (argc > 1) target_id = atoi(argv[1]);

    SerialPC port;
    char dev[] = "/dev/ttyUSB2";
    port.setup(dev, 115200);
    std::cout << "Waiting 8s for ESP32 setup..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));
    tcflush(port.getFd(), TCIFLUSH);

    char buf[16];
    std::string nl = "\n";

    // Simple sine trajectory on one motor
    std::cout << "Sending sine trajectory to Motor " << target_id
              << " (amplitude=0.5 rad, 10s)..." << std::endl;

    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();
        if (t > 10.0) break;

        double theta = 0.5 * sin(2.0 * M_PI * 0.2 * t);  // 0.2 Hz sine
        int theta_mrad = (int)(theta * 1000);

        snprintf(buf, sizeof(buf), "%d", target_id);
        port.write(std::string(buf) + nl);
        snprintf(buf, sizeof(buf), "%d", theta_mrad);
        port.write(std::string(buf) + nl);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Return to 0
    for (int i = 0; i < 50; i++) {
        snprintf(buf, sizeof(buf), "%d", target_id);
        port.write(std::string(buf) + nl);
        port.write(std::string("0") + nl);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Done." << std::endl;
    return 0;
}
