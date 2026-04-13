#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include "SerialLinux.h"

int main() {
    SerialPC port;
    char dev[] = "/dev/ttyUSB2";
    port.setup(dev, 115200);

    char buf[16];
    std::string nl = "\n";

    // Send commands ONLY to motor 2, same as the trajectory script does for motor 1
    // Use the same big position that worked for motor 1 (~0.8 rad = 48 deg)
    std::cout << "Sending to Motor 2 only (pos = -500 mrad)..." << std::endl;

    for (int i = 0; i < 500; i++) {
        snprintf(buf, sizeof(buf), "2");  port.write(std::string(buf) + nl);
        snprintf(buf, sizeof(buf), "-500"); port.write(std::string(buf) + nl);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Done. Did Motor 2 move?" << std::endl;
    return 0;
}
