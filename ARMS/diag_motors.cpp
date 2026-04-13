#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include "SerialLinux.h"

using std::cout;
using std::endl;

void send_cmd(SerialPC &port, int motor_id, float pos_rad) {
    char buf[16];
    std::string nl = "\n";

    snprintf(buf, sizeof(buf), "%d", motor_id);
    std::string id_msg = std::string(buf) + nl;
    port.write(id_msg);

    int theta_int = (int)(pos_rad * 1000);
    snprintf(buf, sizeof(buf), "%d", theta_int);
    std::string theta_msg = std::string(buf) + nl;
    port.write(theta_msg);

    tcdrain(port.getFd());
    cout << "  Sent motor " << motor_id << " -> " << pos_rad << " rad (" << theta_int << ")" << endl;
}

int main() {
    SerialPC port;
    char portname[] = "/dev/ttyUSB2";
    port.setup(portname, 115200);

    cout << "=== MOTOR DIAGNOSTIC ===" << endl;
    cout << "Resetting ESP32 and waiting 10s for setup()..." << endl;

    // Explicit DTR/RTS toggle to guarantee ESP32 resets
    port.resetESP32(10);

    float test_pos = 0.05;  // small 0.05 rad (~3 degrees)

    cout << "\n--- Phase 1: Send each motor to 0 (one at a time, 500ms apart) ---" << endl;
    for (int id = 1; id <= 3; id++) {
        send_cmd(port, id, 0.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    cout << "\n--- Phase 2: Move motor 1 only ---" << endl;
    cout << "  (Watch: does motor 1 move?)" << endl;
    for (int i = 0; i < 20; i++) {
        float pos = test_pos * sin(i * 0.3);
        send_cmd(port, 1, pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cout << "\n--- Phase 3: Move motor 2 only ---" << endl;
    cout << "  (Watch: does motor 2 move?)" << endl;
    for (int i = 0; i < 20; i++) {
        float pos = test_pos * sin(i * 0.3);
        send_cmd(port, 2, pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cout << "\n--- Phase 4: Move motor 3 only ---" << endl;
    cout << "  (Watch: does motor 3 move?)" << endl;
    for (int i = 0; i < 20; i++) {
        float pos = test_pos * sin(i * 0.3);
        send_cmd(port, 3, pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cout << "\n--- Phase 5: All 3 motors together (slow, 200ms between each) ---" << endl;
    for (int i = 0; i < 30; i++) {
        float pos = test_pos * sin(i * 0.2);
        send_cmd(port, 1, pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_cmd(port, 2, -pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        send_cmd(port, 3, pos * 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    cout << "\n--- Phase 6: All 3 motors together (fast, no delay) ---" << endl;
    for (int i = 0; i < 100; i++) {
        float pos = test_pos * sin(i * 0.2);
        send_cmd(port, 1, pos);
        send_cmd(port, 2, -pos);
        send_cmd(port, 3, pos * 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cout << "\n=== DIAGNOSTIC COMPLETE ===" << endl;
    cout << "Results:" << endl;
    cout << "  Phase 2 (M1 only): Did motor 1 oscillate?" << endl;
    cout << "  Phase 3 (M2 only): Did motor 2 oscillate?" << endl;
    cout << "  Phase 4 (M3 only): Did motor 3 oscillate?" << endl;
    cout << "  Phase 5 (all, slow): Did all 3 move?" << endl;
    cout << "  Phase 6 (all, fast): Did all 3 move?" << endl;

    return 0;
}
