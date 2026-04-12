/************************************************************************************
 This program write the string "hello" to the defined COM port.
 This code is modified from
 https://stackoverflow.com/questions/25590539/sending-data-through-serial-port-in-c

 In order to run this program, you need to build it first. At the menu
 bar at the top of VS Code, go to Terminal -> Run Build Task...
 Then click on the option that has 'g++' in it. You will see that a 
 .exe file has been created in its parent folder.

 To run this executable file, go to its parent folder. In the
 directory bar, type in 'cmd' and press Enter. Then, type '.\filename'
 to run the .exe file.

 By: Sai Hein Si Thu   
 Updated: 2023-FEB-04
************************************************************************************/

#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <chrono>      //for working w time
#include <thread>      //for sleeping
#include <iostream>    //cout
#include <iomanip>     //setprecision
//#include <string.h>
#include <cstring>
#include <stdlib.h>    //for itoa()
#include "SerialPC.h"  //my header file

using namespace std::chrono_literals;
using namespace std::chrono;


using std::cout;
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

// These are Eddie's trajectory constants.
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
float w  = 0.1879 * 10;
float theta_d = 0;


int main() {
    // DEBUG
    /* auto test_now = steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto test_later = steady_clock::now();
    duration test_elapsed = test_later - test_now;
    auto test_elapsed_time = test_elapsed.count(); */


    //Setting up motor constants; can delete constants later
    //int kp = 60;   // 20 and 4 pair also gives almost no overshoot, which is good
    //int kd = 4;
    int step = 0;  // keeps track of the number of commands sent


    // Set up for Serial communication //
    SerialPC writePort;
    char write_port_num[] = "COM3";
    int baudrate = 56600;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method

    // For writing command theta to ESP
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    char buf[10];


    // Motor homing (commented out) //
    double theta_i = 0;  //[rad]
    double theta_f = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                a6*cos(6*0*w) + b6*sin(6*0*w));
    double set_homing_dur = 5;            //[seconds]
    double period = set_homing_dur * 2;   //[seconds]
    double homing_w = 2 * 3.14 / period;  //[rad/s]
    double theta_hom;                     //[rad]*/

    int dt = 8;                     //this is an int; units in ms
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable
    //auto dt_ms = milliseconds(dt);

    //double delta_t = dt_ms.count();  // DEBUG
    auto t_pre_loop = steady_clock::now(); 
    auto t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
    auto t_now = t_pre_loop;

    /* while(step <= set_homing_dur / (dt*0.001)) {
        t_now = steady_clock::now();
        //if(1) {  //DEBUG
        if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
            theta_hom = theta_f + (theta_i - theta_f) * ((1 + sin(homing_w * t + 3.14 / 2.0)) / 2.0);
            
            int rounded_theta = theta_hom * 1000;
            cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << endl;  // DEBUG

            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            t_prev = t_now;
            step++;
        }   
    }


    // Idle before walking //
    auto idle_time = seconds(2);
    while(duration_cast<seconds>(steady_clock::now() - t_prev) < idle_time){} */


    // Setting up timing for walking trajectory //
    int run_time = (1 / w) * 40 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
    step = 0;  //reset value of step

    cout << "Sending commands..." << endl;

    // DEBUG
    double sum = 0;
    double max_elapsed = 0;
    double t_elapsed = 0;

    //Calculating and sending commands
    //while(duration_cast<milliseconds>(steady_clock::now() - t_pre_loop) <= run_time_ms) {  //originally duration_cast<seconds>(...)
    auto corrected_t = steady_clock::now();
    while(running_for <= run_time_ms) {
        //t_now = steady_clock::now();  //get curr time at the end instead
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev); //updated
        //cout << elapsed.count() << endl;  // DEBUG

        //if (duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
        if(elapsed >= dt_ms) {
            double t = (dt*pow(10,-3)) * step;  //dt is int but t is still ends up a double, verified
            //cout << elapsed.count() << "\t" << t << endl;  // DEBUG

            // DEBUG
            //t_elapsed = elapsed.count();
            //sum += t_elapsed;
            //if (t_elapsed > max_elapsed) max_elapsed = t_elapsed;
                

            theta_d = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w));

            //cout << theta_d << std::fixed << std::setprecision(3) << "," << endl;  // DEBUG
            cout << theta_d << std::fixed << std::setprecision(3) << endl;  // DEBUG
            //cout << elapsed.count() << std::fixed << std::setprecision(5) << endl;  // DEBUG

            int rounded_theta = theta_d * 1000;
            //cout << rounded_theta << " step: " << step << endl;

            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            
            // DEBUG
            //num_bytes = sizeof(theta_char) / sizeof(theta_char[0]);
            //cout << theta_d << endl;

            writePort.write(theta_str_n);
            //cout << t << endl;
            //t_prev = t_now;                              //before: set t_prev to actual previous time
            t_prev = corrected_t + milliseconds(dt*step);  //after: t_prev is set to previous element of time array
            

            // DOES NOT WORK
            //auto increment = std::chrono::duration<float, std::milli>(dt * step);
            //t_prev = corrected_t + increment; // after: but with <float, milli> implementation; 
            step++;
        }

        t_now = steady_clock::now();
        running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
        //running_for = duration_cast<milliseconds>(t_now - t_pre_loop); //even tho int64 w this one, it's a decimal

        //cout << "ignore this" << endl;  // DEBUG
    }


    cout << endl << "Commands sent!\n";

    // DEBUG, dt_ms.count() = 8
    //cout << "Max time elapsed = " << max_elapsed << endl;
    //cout << "Error (%): " << (sum - step*dt_ms.count()) / (step*dt_ms.count()) * 100 << " percent" << endl;  
    
    //cout << "step = " << step << endl;  // DEBUG
}