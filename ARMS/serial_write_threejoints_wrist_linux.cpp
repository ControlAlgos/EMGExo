#include <stdio.h>
#include <math.h>
#include <chrono>      //for working w time
#include <thread>      //for sleeping
#include <iostream>    //cout
#include <iomanip>     //setprecision
#include <cstring>
#include <stdlib.h>
#include "SerialLinux.h"

using namespace std::chrono_literals;
using namespace std::chrono;

using std::cin;
using std::cout;
using std::endl;


/*******  Constants  *******/
// These constants match the AK80-64 motor. Change these to match your motor.
const float P_MIN = -12.5;     // Position range (rad) — typically unchanged unless your joint limits are tighter
const float P_MAX = 12.5;
const float V_MIN = -30.0;     // Velocity range (rad/s)
const float V_MAX = 30.0;
const float T_MIN = -18.0;     // Torque range (Nm)
const float T_MAX = 18.0;
const float Kp_MIN = 0.0;
const float Kp_MAX = 500.0;
const float Kd_MIN = 0.0;
const float Kd_MAX = 5.0;
const float Test_Pos = 0.0;

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
float w  = 0.65; //0.1879 * 10;  //removed *10 that was there for testing
float theta_d_right = 0;
float theta_d_left = 0;

// Walking (Knee joint) 
float ka0 = 25.7;
float ka1 = -3.83;
float kb1 = -19.28;
float ka2 = -8.54;
float kb2 = 17.93;
float ka3 = 1.91;
float kb3 = 3.77;
float ka4 = 1.09;
float kb4 = 1.50;
float ka5 = 2.05;
float ka6 = -0.31;
float kb6 = -0.90;
float kb5 = 0.58;
float kw = 2.8/2; //0.03886 * 50;  // period = 100 (unitless)
float ktheta_d_right = 0;
float ktheta_d_left = 0;



// Left-Right Hip Trajectory adjustments
float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift/360 * traj_period;  //phase shift in units of sec;
float amp_shift_deg = 23;  //20 deg amp_shift
//float amp_shift_rad = amp_shift_deg * M_PI/180;  //convert to rad, not used

int main() {
    int step;  // keeps track of the number of commands sent

    // ========== Set up for Serial communication ========== //
    // Write definition and setup
    SerialPC writePort;
    char write_port_num[] = "/dev/ttyUSB2"; // CP210x USB to UART Bridge
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    std::string id_str_n;
    char buf[10];

    // Motor IDs (Four Joints)
    const int NUM_OF_MOTORS = 3;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 1, 2, 3};  //right (hip), left (hip), right (knee), Left(knee) 

    for (int i = 1; i <= NUM_OF_MOTORS; ++i) {
        itoa(id[i], buf, 10);  
        id_str_n = buf + newline;
        writePort.write(id_str_n);

        int zero_effort = 0;
        itoa(zero_effort, buf, 10);
        theta_str_n = buf + newline;
        writePort.write(theta_str_n);
    }

    // Give motors a moment to settle into motor mode
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // or 100ms

        step = 0;

        // ========== Motor HOMING sequence ========== // (HIPS)++++++++++++++++++++++++++++++++++
        double theta_hom;                     //[rad]
        double theta_i = 0;                   //[rad]
        double theta_f_right = (M_PI/180) * 1.25*(a0 + a1*cos(0*w) + b1*sin(0*w) + 
                    a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                    a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                    a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg);  //[rad]; the multiplier pi/180 does not make sense lol, will fix it later
        
        double theta_f_left = (-M_PI/180) * 2*(a0 + a1*cos((0-b)*w) + b1*sin((0-b)*w) + 
                a2*cos(2*(0-b)*w) + b2*sin(2*(0-b)*w) + a3*cos(3*(0-b)*w) + b3*sin(3*(0-b)*w) + 
                a4*cos(4*(0-b)*w) + b4*sin(4*(0-b)*w) + a5*cos(5*(0-b)*w) + b5*sin(5*(0-b)*w) +
                a6*cos(6*(0-b)*w) + b6*sin(6*(0-b)*w) - amp_shift_deg);

        
         // ========== Motor HOMING sequence ========== // (KNEE)
        double ktheta_f_right= (-M_PI/180) * 7*(ka0 + ka1*cos(0*w) + kb1*sin(0*w) + 
                    ka2*cos(2*0*w) + kb2*sin(2*0*w) + ka3*cos(3*0*w) + kb3*sin(3*0*w) + 
                    ka4*cos(4*0*w) + kb4*sin(4*0*w) + ka5*cos(5*0*w) + kb5*sin(5*0*w) - amp_shift_deg);

        double ktheta_f_left = (-M_PI/180) * (ka0 + ka1*cos((0-b)*w) + kb1*sin((0-b)*w) + 
                ka2*cos(2*(0-b)*w) + kb2*sin(2*(0-b)*w) + ka3*cos(3*(0-b)*w) + kb3*sin(3*(0-b)*w) + 
                ka4*cos(4*(0-b)*w) + kb4*sin(4*(0-b)*w) + ka5*cos(5*(0-b)*w) + kb5*sin(5*(0-b)*w) - amp_shift_deg);
        

        double set_homing_dur = 5;        //[seconds]
        double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
        double homing_w = 2 * M_PI / period;  //[rad/s]

        //double delta_t = dt_ms.count();  // DEBUG
        int dt = 10;    //this is an int; units in ms
        auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable
        auto t_pre_loop = steady_clock::now(); 
        auto t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
        auto t_now = t_pre_loop;
        auto corrected_t = steady_clock::now();

        while(step <= set_homing_dur / (dt*0.001)) {
            t_now = steady_clock::now();
            if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
                double t = step * dt * 0.001;

// ===== Send 1st hip motor's ID =========================================================== //
                itoa(id[1], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << std::fixed << std::setprecision(3) << ";";

                // Formatting data
                int rounded_theta = theta_hom * 1000;
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

// ===== Send 2nd hip motor's ID ========================================================== //
                itoa(id[2], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

// ===== Send 3rd hip motor's ID ========================================================== //
                itoa(id[3], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = ktheta_f_right + (theta_i - ktheta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

                t_prev = corrected_t + milliseconds(dt*step);
                step++;  //del later cuz there's one below
            } //del later
        } //del later

    // Setting up timing for walking trajectory //
    t_pre_loop = steady_clock::now(); 
    t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
    t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
    step = 0;  //reset value of step

    //Calculating and sending commands
    corrected_t = steady_clock::now();
    while(running_for <= run_time_ms) {
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev); //updated

        if(elapsed >= dt_ms) {
            double t = (dt*pow(10,-3)) * step;  //dt is int but t is still ends up a double, verified
            
// ===== RADIAL/ULNAR DEVIATION =================================================================== //
            itoa(id[1], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_d_right = (M_PI/180) * 1.25*(a0 + a1*cos(t*w) + b1*sin(t*w) + 
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            cout << theta_d_right << std::fixed << std::setprecision(3) << ";"; // DEBUG

            int rounded_theta = theta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            

// ===== FLEXION/EXTENSION ================================================================== //
            itoa(id[2], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_d_left = (-M_PI/180) * 2*(a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) + 
                a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) + 
                a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
                a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);
            
            cout << theta_d_left << std::fixed << std::setprecision(3) << ";";

            rounded_theta = theta_d_left * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

// ===== PRONATION/SUPINATION (AXIAL ROTATION) =================================================================== //
            itoa(id[3], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            ktheta_d_right = (-M_PI/180) * 7*(ka0 + ka1*cos(t*w) + kb1*sin(t*w) + 
                ka2*cos(2*t*w) + kb2*sin(2*t*w) + ka3*cos(3*t*w) + kb3*sin(3*t*w) + 
                ka4*cos(4*t*w) + kb4*sin(4*t*w) + ka5*cos(5*t*w) + kb5*sin(5*t*w) +
                ka6*cos(6*t*w) + kb6*sin(6*t*w) - amp_shift_deg);

            cout << ktheta_d_right << std::fixed << std::setprecision(3) << ";"; // DEBUG

            rounded_theta = ktheta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            
            t_prev = corrected_t + milliseconds(dt*step);  //after: t_prev is set to previous element of time array
             
            step++;
        }
        t_now = steady_clock::now();
        running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated   
    }
    cout << endl << "Commands sent!\n";
}