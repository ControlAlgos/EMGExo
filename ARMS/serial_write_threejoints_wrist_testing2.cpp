#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <stdlib.h>
#include "SerialPC.h"

using namespace std::chrono_literals;
using namespace std::chrono;
using std::cin;
using std::cout;
using std::endl;

/*******  Constants *******/
const float P_MIN = -12.5;
const float P_MAX = 12.5;
const float V_MIN = -30.0;
const float V_MAX = 30.0;
const float T_MIN = -18.0;
const float T_MAX = 18.0;
const float Kp_MIN = 0.0;
const float Kp_MAX = 500.0;
const float Kd_MIN = 0.0;
const float Kd_MAX = 5.0;

float a0 = 40.69, a1 = 23.22, b1 = -8.65, a2 = -4.487, b2 = 3.338, a3 = 0.3995, b3 = 1.389;
float a4 = 0.7047, b4 = 0.7989, a5 = 1.078, b5 = 0.342, a6 = -0.2732, b6 = 0.06696, w = 0.65;
float ka0 = 25.7, ka1 = -3.83, kb1 = -19.28, ka2 = -8.54, kb2 = 17.93, ka3 = 1.91, kb3 = 3.77;
float ka4 = 1.09, kb4 = 1.50, ka5 = 2.05, kb5 = 0.58, ka6 = -0.31, kb6 = -0.90, kw = 1.4;

float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift / 360.0 * traj_period;
float amp_shift_deg = 23;

int main() {
    SerialPC writePort;
    char write_port_num[] = "COM3";
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);
    char buf[10];
    std::string newline = "\n", theta_str_n, id_str_n;
    const int NUM_OF_MOTORS = 3;
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 1, 2, 3};

    // ========== Step 1: Soft Start to Target Pos ==========
    double theta_f_right = (M_PI/180) * 1.25 * (a0 + a1 + a2 + a3 + a4 + a5 + a6 - amp_shift_deg);
    double theta_f_left = (-M_PI/180) * 2 * (a0 + a1*cos(-b*w) + b1*sin(-b*w) + a2*cos(-2*b*w) +
                            b2*sin(-2*b*w) + a3*cos(-3*b*w) + b3*sin(-3*b*w) + a4*cos(-4*b*w) +
                            b4*sin(-4*b*w) + a5*cos(-5*b*w) + b5*sin(-5*b*w) + a6*cos(-6*b*w) +
                            b6*sin(-6*b*w) - amp_shift_deg);
    double ktheta_f_right = (-M_PI/180) * 7 * (ka0 + ka1 + ka2 + ka3 + ka4 + ka5 + ka6 - amp_shift_deg);

    int rounded_theta;

    // Hip Right
    itoa(id[1], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
    rounded_theta = theta_f_right * 1000; itoa(rounded_theta, buf, 10);
    theta_str_n = buf + newline; writePort.write(theta_str_n);

    // Hip Left
    itoa(id[2], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
    rounded_theta = theta_f_left * 1000; itoa(rounded_theta, buf, 10);
    theta_str_n = buf + newline; writePort.write(theta_str_n);

    // Knee Right
    itoa(id[3], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
    rounded_theta = ktheta_f_right * 1000; itoa(rounded_theta, buf, 10);
    theta_str_n = buf + newline; writePort.write(theta_str_n);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ========== Step 2: Homing Fade-In ==========
    int step = 0;
    double set_homing_dur = 5.0;
    double period = set_homing_dur * 2;
    double homing_w = 2 * M_PI / period;
    int dt = 10;
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);
    auto t_pre_loop = steady_clock::now(), t_prev = t_pre_loop, t_now = t_pre_loop;

    while (step <= set_homing_dur / (dt * 0.001)) {
        t_now = steady_clock::now();
        if (duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            //Radial/Ulnar Deviation
            double theta_hom = theta_f_right;
            itoa(id[1], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            rounded_theta = theta_hom * 1000; itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline; writePort.write(theta_str_n);

            //Flexion/Extension
            theta_hom = theta_f_left;
            itoa(id[2], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            rounded_theta = theta_hom * 1000; itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline; writePort.write(theta_str_n);

            //Supination/Pronation
            theta_hom = ktheta_f_right;
            itoa(id[3], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            rounded_theta = theta_hom * 1000; itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline; writePort.write(theta_str_n);

            t_prev = t_now;
            step++;
        }
    }

    // ========== Step 3: Run Cyclic Trajectory ==========
    step = 0;
    int run_time = static_cast<int>((2 * M_PI / w) * 5 * 1000);
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    t_pre_loop = steady_clock::now(); t_prev = t_pre_loop; t_now = t_pre_loop;
    auto running_for = std::chrono::duration<float, std::milli>(0);

    cout << "\nRunning sinusoidal trajectory...\n\n";

    while (running_for <= run_time_ms) {
        t_now = steady_clock::now();
        if (duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            // Radial/Ulnar Deviation
            double theta_d_right = (M_PI/180) * 1.25 * (a0 + a1*cos(t*w) + b1*sin(t*w) +
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) +
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);
            rounded_theta = theta_d_right * 1000;
            double theta_deg_right = theta_d_right * (180.0 / M_PI);
            cout << std::fixed << std::setprecision(2)
                 << "t=" << t << " s | Radial and Ulnar Deviation: " << theta_deg_right << " deg";

            itoa(id[1], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            itoa(rounded_theta, buf, 10); theta_str_n = buf + newline; writePort.write(theta_str_n);

            //Flexion/Extension
            double theta_d_left = (-M_PI/180) * 2 * (a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) +
                a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) +
                a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
                a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);
            rounded_theta = theta_d_left * 1000;
            double theta_deg_left = theta_d_left * (180.0 / M_PI);
            cout << " | Flexion and Extension: " << theta_deg_left << " deg";

            itoa(id[2], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            itoa(rounded_theta, buf, 10); theta_str_n = buf + newline; writePort.write(theta_str_n);

            //Supination/Pronation
            double ktheta_d_right = (-M_PI/180) * 7 * (ka0 + ka1*cos(t*w) + kb1*sin(t*w) +
                ka2*cos(2*t*w) + kb2*sin(2*t*w) + ka3*cos(3*t*w) + kb3*sin(3*t*w) +
                ka4*cos(4*t*w) + kb4*sin(4*t*w) + ka5*cos(5*t*w) + kb5*sin(5*t*w) +
                ka6*cos(6*t*w) + kb6*sin(6*t*w) - amp_shift_deg);
            rounded_theta = ktheta_d_right * 1000;
            double ktheta_deg_right = ktheta_d_right * (180.0 / M_PI);
            cout << " | Pronation and Supination: " << ktheta_deg_right << " deg\n";

            itoa(id[3], buf, 10); id_str_n = buf + newline; writePort.write(id_str_n);
            itoa(rounded_theta, buf, 10); theta_str_n = buf + newline; writePort.write(theta_str_n);

            step++;
            t_prev = t_now;
        }
        running_for = t_now - t_pre_loop;
    }

    cout << "\nCommands sent!\n";
}