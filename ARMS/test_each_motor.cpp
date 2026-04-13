#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <termios.h>
#include "SerialLinux.h"

using std::cout;
using std::endl;

void send_cmd(SerialPC &port, int id, int theta_mrad) {
    char buf[16];
    std::string nl = "\n";
    snprintf(buf, sizeof(buf), "%d", id);
    port.write(std::string(buf) + nl);
    snprintf(buf, sizeof(buf), "%d", theta_mrad);
    port.write(std::string(buf) + nl);
}

int main() {
    SerialPC writePort;
    char port[] = "/dev/ttyUSB2";
    writePort.setup(port, 115200);

    cout << "\n>>> Press the RESET button on the ESP32 now. <<<" << endl;
    cout << ">>> Wait for motor beeps (~8 sec), then press ENTER here. <<<\n" << endl;
    std::cin.get();
    tcflush(writePort.getFd(), TCIFLUSH);

    int ids[] = {1, 2, 3};

    // First send zero to all motors to confirm they're alive
    cout << "Sending pos=0 to all motors..." << endl;
    for (int i = 0; i < 3; i++) {
        send_cmd(writePort, ids[i], 0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Test each motor individually — 500 mrad (0.5 rad ≈ 28.6 deg), clearly visible
    int test_pos[] = {500, -500, 500};  // mrad per motor (positive/negative to match joint dirs)
    for (int i = 0; i < 3; i++) {
        const char* names[] = {"Radial/Ulnar", "Flexion/Extension", "Pronation/Supination"};
        cout << "\n========================================" << endl;
        cout << "Testing Motor " << ids[i] << " (" << names[i] << "): moving to "
             << test_pos[i]/1000.0 << " rad (" << test_pos[i]/1000.0*57.3 << " deg)" << endl;
        cout << "  >>> WATCH THIS MOTOR <<<" << endl;
        cout << "========================================" << endl;

        // Ramp to position over 1s, hold 2s, ramp back over 1s
        for (int s = 0; s <= 100; s++) {
            int pos = test_pos[i] * s / 100;
            send_cmd(writePort, ids[i], pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        cout << "  Holding..." << endl;
        for (int s = 0; s < 200; s++) {
            send_cmd(writePort, ids[i], test_pos[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        cout << "  Returning to 0..." << endl;
        for (int s = 100; s >= 0; s--) {
            int pos = test_pos[i] * s / 100;
            send_cmd(writePort, ids[i], pos);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        cout << "  Pausing 2s before next motor..." << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    // All 3 interleaved
    cout << "\n========================================" << endl;
    cout << "Testing ALL 3 interleaved: ramping to 0.5 rad" << endl;
    cout << "========================================" << endl;
    for (int s = 0; s <= 100; s++) {
        send_cmd(writePort, 1, 500 * s / 100);
        send_cmd(writePort, 2, -500 * s / 100);
        send_cmd(writePort, 3, 500 * s / 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    cout << "  Holding 3s..." << endl;
    for (int s = 0; s < 300; s++) {
        send_cmd(writePort, 1, 500);
        send_cmd(writePort, 2, -500);
        send_cmd(writePort, 3, 500);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    cout << "\nReturning all to 0..." << endl;
    for (int s = 100; s >= 0; s--) {
        send_cmd(writePort, 1, 500 * s / 100);
        send_cmd(writePort, 2, -500 * s / 100);
        send_cmd(writePort, 3, 500 * s / 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cout << "\nDone." << endl;
    return 0;
}
