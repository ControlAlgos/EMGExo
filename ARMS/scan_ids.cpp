#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdlib.h>
#include "SerialLinux.h"

int main() {
    SerialPC port;
    char dev[] = "/dev/ttyUSB2";
    port.setup(dev, 115200);

    char buf[16];
    std::string nl = "\n";

    // Try each ID from 1 to 6 — watch which physical motor moves
    for (int id = 1; id <= 6; id++) {
        std::cout << "\n=== Sending to ID " << id << " (0.5 rad for 3s) — WATCH MOTORS ===" << std::endl;

        for (int i = 0; i < 300; i++) {
            snprintf(buf, sizeof(buf), "%d", id);
            port.write(std::string(buf) + nl);
            port.write(std::string("500") + nl);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Return to 0
        for (int i = 0; i < 100; i++) {
            snprintf(buf, sizeof(buf), "%d", id);
            port.write(std::string(buf) + nl);
            port.write(std::string("0") + nl);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "=== ID " << id << " done. Pause 2s ===" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    std::cout << "\nWhich IDs moved? Tell me." << std::endl;
    return 0;
}
