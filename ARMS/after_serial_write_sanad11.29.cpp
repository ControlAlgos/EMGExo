#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <chrono>      //for working w time
#include <thread>      //for sleeping
#include <iostream>    //cout
#include <iomanip>     //setprecision
#include <cstring>
#include <stdlib.h>    //for itoa()
#include "SerialPC.h"  //my header file
#include "nlohmann/json.hpp" //JSON reading header file, change location accordingly
#include <fstream>
#include <unistd.h>
#include <string.h>

using json = nlohmann::json;
using namespace std;
using namespace std::chrono_literals;
using namespace std::chrono;

using std::cin;
using std::cout;
using std::cerr;
using std::endl;

/*******  Constants  *******/
// These constants match the AK80-64 motor. Change these to match your motor.
const float P_MIN = -12.5;
const float P_MAX = 12.5;
const float V_MIN = -8;
const float V_MAX = 8;
const float T_MIN = -144;
const float T_MAX = 144;
const float Kp_MIN = 0;
const float Kp_MAX = 500;
const float Kd_MIN = 0;
const float Kd_MAX = 5;
const float Test_Pos= 0.0;

// These are Eddie's trajectory constants for hip joint when walking
float a0 = 40.69;
float a1 = 23.22;
float b1 = -8.65;
float a2 = -4.487;
float b2 = 3.338;
float a3 = 0.3995;
float b3 = 1.389;
float a4 = 0.7047;
float b4 = 0.7989;
float a5 = 1.078;
float b5 = 0.342;
float a6 = -0.2732;
float b6 = 0.06696;
float w  = 0.4;  //removed *10 that was there for testing
float theta_d_right = 0;
float theta_d_left = 0;
float prev_theta_d_right = 0;
float prev_theta_d_left = 0;

// Left-Right Hip Trajectory adjustments
float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift/360 * traj_period;  //phase shift in units of sec;
float amp_shift_deg = 23;  //20 deg amp_shift
//float amp_shift_rad = amp_shift_deg * M_PI/180;  //convert to rad, not used

int main() {
    int step;  // keeps track of the number of commands sent

    SerialPC writePort;
    char write_port_num[] = "COM3"; // CP210x USB to UART Bridge (COM3) Option on the computer
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    std::string id_str_n;
    char buf[10];

    const int NUM_OF_MOTORS = 2;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 3, 4};  //right, left
    
    step = 0;

    // ========== Motor homing sequence ========== //
    double theta_hom;                     //[rad]
    double theta_i = 0;                   //[rad]
    double theta_f_right = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg);  //[rad]; the multiplier pi/180 does not make sense lol, will fix it later
    
    double theta_f_left = (-M_PI/180) * (a0 + a1*cos((0-b)*w) + b1*sin((0-b)*w) + 
            a2*cos(2*(0-b)*w) + b2*sin(2*(0-b)*w) + a3*cos(3*(0-b)*w) + b3*sin(3*(0-b)*w) + 
            a4*cos(4*(0-b)*w) + b4*sin(4*(0-b)*w) + a5*cos(5*(0-b)*w) + b5*sin(5*(0-b)*w) +
            a6*cos(6*(0-b)*w) + b6*sin(6*(0-b)*w) - amp_shift_deg);

    double set_homing_dur = 10;        //[seconds]
    double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
    double homing_w = 2 * M_PI / period;  //[rad/s]

    int dt = 2;    //this is an int; units in ms
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable
    auto t_pre_loop = steady_clock::now(); 
    auto t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
    auto t_now = t_pre_loop;
    auto corrected_t = steady_clock::now();

    while(step <= set_homing_dur / (dt*0.001)) {
        t_now = steady_clock::now();
        if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            itoa(id[3], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_hom = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
            cout << theta_hom << std::fixed << std::setprecision(3) << ";";

            int rounded_theta = theta_hom * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

            itoa(id[4], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_hom = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
            cout << theta_hom << ";";

            rounded_theta = theta_hom * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

            t_prev = corrected_t + milliseconds(dt * step);
            step++;
        }
    }

    // Setting up timing for walking trajectory //
    t_pre_loop = steady_clock::now(); 
    t_prev = t_pre_loop;
    t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
    step = 0;  //reset value of step

    corrected_t = steady_clock::now();
    t_now = corrected_t;  // Initialize t_now

    while (running_for <= run_time_ms) {
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev);

        if (elapsed >= dt_ms) {
            double t = (dt * pow(10, -3)) * step;

            itoa(id[3], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            ifstream motor;
            bool motorFileOpened = false;

            while (!motorFileOpened) {
                motor.open("C:\\Users\\Mo Sharifi\\Documents\\PlatformIO\\Projects\\Exoskeleton code\\serial-rw\\serialComm_WriteAndRead");

                if (!motor.is_open()) {
                    cerr << "Error: Unable to open the JSON file. Retrying..." << endl;
                    std::this_thread::sleep_for(100ms); // Add a delay before retrying
                } else {
                    motorFileOpened = true;
                }
            }

            try {
                json jf = json::parse(motor);
                string motorCaseString = jf["case"];
                string freqString = jf["w"];
                int motorCase = stoi(motorCaseString);
                float freq = stof(freqString);

                if (motorCase > 0) {
                    theta_d_right = (M_PI / 180) * (a0 + a1 * cos(t * w) + b1 * sin(t * w) +
                        a2 * cos(2 * t * w) + b2 * sin(2 * t * w) + a3 * cos(3 * t * w) + b3 * sin(3 * t * w) +
                        a4 * cos(4 * t * w) + b4 * sin(4 * t * w) + a5 * cos(5 * t * w) + b5 * sin(5 * t * w) +
                        a6 * cos(6 * t * w) + b6 * sin(6 * t * w) - amp_shift_deg);

                    prev_theta_d_right = theta_d_right;

                    cout << theta_d_right << std::fixed << std::setprecision(3) << ";";

                    int rounded_theta = theta_d_right * 1000;
                    itoa(rounded_theta, buf, 10);
                    theta_str_n = buf + newline;
                    writePort.write(theta_str_n);

                    itoa(id[4], buf, 10);
                    id_str_n = buf + newline;
                    writePort.write(id_str_n);

                    theta_d_left = (-M_PI / 180) * (a0 + a1 * cos((t - b) * w) + b1 * sin((t - b) * w) +
                        a2 * cos(2 * (t - b) * w) + b2 * sin(2 * (t - b) * w) + a3 * cos(3 * (t - b) * w) + b3 * sin(3 * (t - b) * w) +
                        a4 * cos(4 * (t - b) * w) + b4 * sin(4 * (t - b) * w) + a5 * cos(5 * (t - b) * w) + b5 * sin(5 * (t - b) * w) +
                        a6 * cos(6 * (t - b) * w) + b6 * sin(6 * (t - b) * w) - amp_shift_deg);

                    prev_theta_d_left = theta_d_left;

                    cout << theta_d_left << std::fixed << std::setprecision(3) << ";";

                    rounded_theta = theta_d_left * 1000;
                    itoa(rounded_theta, buf, 10);
                    theta_str_n = buf + newline;
                    writePort.write(theta_str_n);

                    t_prev = t_now;
                    step++;
                }

                if (motorCase == 0) {
                    theta_d_right = prev_theta_d_right;

                    cout << theta_d_right << std::fixed << std::setprecision(3) << ";";

                    int rounded_theta = theta_d_right * 1000;
                    itoa(rounded_theta, buf, 10);
                    theta_str_n = buf + newline;
                    writePort.write(theta_str_n);

                    itoa(id[4], buf, 10);
                    id_str_n = buf + newline;
                    writePort.write(id_str_n);

                    theta_d_left = prev_theta_d_left;

                    cout << theta_d_left << std::fixed << std::setprecision(3) << ";";

                    rounded_theta = theta_d_left * 1000;
                    itoa(rounded_theta, buf, 10);
                    theta_str_n = buf + newline;
                    writePort.write(theta_str_n);

                    t_prev = t_now;
                }
            } catch (const json::parse_error &e) {
                cerr << "Error while parsing JSON: " << e.what() << endl;
                // Handle the error or exit the program.
            } catch (const std::exception &e) {
                cerr << "Error: " << e.what() << endl;
                // Handle other exceptions.
            }

            motor.close(); // Close the file after use.
            motor.clear(); // Clear the file stream state.

            t_now = steady_clock::now();
            running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop);

            step++;
        }
    }

    cout << endl << "Commands sent!\n";
}

