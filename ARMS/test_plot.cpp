#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <stdlib.h>
#include <termios.h>
#include "SerialLinux.h"

using namespace std::chrono;
using std::cout;
using std::endl;

int main() {
    SerialPC writePort;
    char port[] = "/dev/ttyUSB2";
    writePort.setup(port, 115200);

    cout << "Waiting 8s for ESP32 setup (motors entering control mode)..." << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));
    tcflush(writePort.getFd(), TCIFLUSH);
    cout << "Ready. Sending commands." << endl;

    char buf[32];
    std::string nl = "\n";

    const int NUM_MOTORS = 3;
    int ids[NUM_MOTORS] = {1, 2, 3};
    double amp = 0.15;    // 0.15 rad ≈ 8.6 deg — small and safe
    double freq = 0.3;    // Hz
    double duration = 10.0; // seconds (3 full cycles)
    int dt_ms = 10;

    cout << "\nSmall-amplitude sine test: " << amp << " rad, "
         << freq << " Hz, " << duration << "s" << endl;
    cout << "Run motor_logger.py in another terminal to capture feedback.\n" << endl;

    auto t0 = steady_clock::now();
    int step = 0;

    while (true) {
        auto now = steady_clock::now();
        double t = duration_cast<microseconds>(now - t0).count() / 1e6;
        if (t > duration) break;

        double theta = amp * sin(2.0 * M_PI * freq * t);
        int theta_mrad = (int)(theta * 1000);

        for (int i = 0; i < NUM_MOTORS; i++) {
            snprintf(buf, sizeof(buf), "%d", ids[i]);
            writePort.write(std::string(buf) + nl);
            snprintf(buf, sizeof(buf), "%d", theta_mrad);
            writePort.write(std::string(buf) + nl);
        }

        if (step % 100 == 0) {
            cout << std::fixed << std::setprecision(2)
                 << "  t=" << t << "s  theta=" << theta << " rad ("
                 << theta * 180.0 / M_PI << " deg)" << endl;
        }
        step++;

        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    }

    // Return to zero smoothly
    cout << "\nReturning to zero..." << endl;
    for (int s = 0; s < 100; s++) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            snprintf(buf, sizeof(buf), "%d", ids[i]);
            writePort.write(std::string(buf) + nl);
            writePort.write(std::string("0") + nl);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cout << "Done. All motors returned to zero." << endl;
    return 0;
}
