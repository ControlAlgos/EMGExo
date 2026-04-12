/*******************************************************************************
 After double-motor implementation!

 Use this program together with '1after_serial_receive_and_control_onESP.cpp'
 on ESP32.
 
 This program runs on a computer and generates a new theta value every dt.
 The theta values can be calculated using any given trajectory equation.
 This program is capable of homing and commanding several motors to follow
 the desired trajectory. To control only one motor, use 'before_serial_write.cpp' 
 (on computer) and '1before_serial_receive_and_control_onESP.cpp' (on ESP32).

 In order to run this program, you need to build it first. At the menu
 bar at the top of VS Code, go to Terminal -> Run Build Task...
 Then click on the custom task named 'BUILD'. After building, you will see that 
 a .exe file has been created in its parent folder.

 Note: the task 'BUILD' is within the .vscode folder so make sure that folder
 is copied.

 To run this executable file, go to its parent folder. In the file
 directory bar, type in 'cmd' and press Enter. Then, type '.\filename'
 to run the .exe file.

 By: Sai Hein Si Thu   
 Updated: 2023-JUN-11
*******************************************************************************/

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

using namespace std::chrono_literals;
using namespace std::chrono;

using std::cin;
using std::cout;
using std::endl;


/**********  Constants  **********/
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
float w  = 0.1879 * 10;  //*10 for faster data collection process, \
                            adjust accordingly for real walking speed
float theta_d_right = 0;
float theta_d_left = 0;


/**********  Global Variables  **********/
// Left-Right Hip Trajectory adjustments
float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift/360 * traj_period;  //phase shift in units of sec;
int step;  // keeps track of the number of commands sent


int main() {
    // ==========  Definition and setup for writing to COM port  ========== //
    SerialPC writePort;
    char write_port_num[] = "COM3";
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);
    char theta_char[] = "";
    std::string newline = "\n", theta_str_n, id_str_n;
    char buf[10];


    // ==========  Definition and setup for reading from COM port  ========== //
    /* 
     If this section can be set up properly, it will allow the program to get
     access to the feedback data. As of now, the feedback data is only printed
     to PuTTY via ESP32's 2nd Serial port and is not directly being read by this
     program. Implementing this section slows down the code and that might have
     to do with the program waiting for something to receive in Serial. Will 
     look further into it.
    */

    //SerialPC readPort;
    //char read_port_num[] = "COM8";
    //readPort.setup(read_port_num, baudrate);
    //const int BYTES_IN = 6;  //change back to 6 if sending floats as string; 4 if floats as binary
    //char enc_str[BYTES_IN];  //if as strings, leave 1 char out for trailing 0: '\0' if initializing it to sth
    //float enc;

    // Storing data
    //const int SIZE = (run_time / dt) + 50;  //will implement dynamic memory alloc later, use fixed sized arrays for now
    //float *enc_data1 = new float[SIZE];
    
    //float enc_data1[1000];
    //float enc_data2[1000];


    // ==========  Sending active motors' IDs to ESP32  ========== //
    // !!! NOT FULLY IMPLEMENTED YET !!! //

    /* 
     Type in the IDs of the active motors and they will be sent to the ESP
     where they'll be put into listening mode.This prevents us from having to 
     rewrite the same motor IDs in ESP32's setup function, specifically in the
     enter_control_mode() func. However, it is not properly implemented yet, 
     so we have to type ID in here and also in ESP32's setup func. 
    */

    const int NUM_OF_MOTORS = 1;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 1};
    
    // Send all connected motors' IDs so that they can all be put into listening mode 
    //for (int i = 0; i <= NUM_OF_MOTORS; i++){
    //    // Pre-processing data to be sent over COM port; modify later to send them as an int maybe
    //    itoa(id[i], buf, 10);
    //    id_str_n = buf + newline;
    //    writePort.write(id_str_n);
    //    cout << id_str_n;
    //}

    // Pause program to let all motors enter listening mode and settle down
    //cout << "wait start" << endl;  
    //std::this_thread::sleep_for(seconds(1));
    //cout << "wait done" << endl;


    // ==========  Motor homing sequence  ========== //
    /*
     This section homes the motor from whatever position it is at to the
     beginning theta of the trajectory. The motor needs to be restarted/power
     cycled so that its encoder resets to 0. [MODIFY: use set_origin() function
     from 1after_serial_receive_and_control_onESP.cpp so that we don't have to
     power cycle the motors.] Power cycling AK10-9 will not reset its encoder 
     to 0 rad since it has an absolute encoder, where the AK80-64, 80-9, and
     70-10 all have relative encoders. 
     
     Variables:
     - double theta_hom = theta var tht updates every dt
              theta_hom = theta_f + (theta_i - theta_f) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
     - double theta_i = current motor pos (Not initial theta of the desired traj) 
     - double theta_f_right = right hip motor's final position after homing, 
                which is the same as the initial theta of the desired traj (t=0)
     - double theta_f_left = similar to theta_f_right 
     - float amp_shift_deg = shifts the entire traj to match a real person's gait
    */
    
    // Calculate start and end theta for homing: theta_i and theta_f
    double theta_i = 0;        //[rad]
    float amp_shift_deg = 23;  //shifts vertically by x deg
    double theta_f_right = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg);  //[rad], t=0
    
    double theta_f_left = (M_PI/180) * (a0 + a1*cos((0-b)*w) + b1*sin((0-b)*w) + 
            a2*cos(2*(0-b)*w) + b2*sin(2*(0-b)*w) + a3*cos(3*(0-b)*w) + b3*sin(3*(0-b)*w) + 
            a4*cos(4*(0-b)*w) + b4*sin(4*(0-b)*w) + a5*cos(5*(0-b)*w) + b5*sin(5*(0-b)*w) +
            a6*cos(6*(0-b)*w) + b6*sin(6*(0-b)*w) - amp_shift_deg);  //[rad], t=0

    // Homing function's characteristics
    double set_homing_dur = 5;            //[seconds]
    double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
    double homing_w = 2 * M_PI / period;  //[rad/s]

    // Variables for timing (used in homing and also in desired traj)
    int dt = 2;  //in ms
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //var type = 'Duration'
    auto t_pre_loop = steady_clock::now();   //get timepoint before while loop below
    auto t_prev = t_pre_loop;  //they're not equal in terms of context
    auto t_now = t_pre_loop;   //just using the equal sign cuz didnt want to write steady_clock::now() multiple times
    auto corrected_t = steady_clock::now();  //will explain below
    
    // Theta var that updates every dt
    double theta_hom;  //[rad]
    step = 0;


    // Motor homing loop
    // Loops until the num of steps represents tht it's been 'set_homing_dur' secs
    while(step <= set_homing_dur / (dt*0.001)) {
        t_now = steady_clock::now();
        if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            // ===== Send 1st hip motor's ID ===== //
            itoa(id[1], buf, 10);  //use the ID at index 1 (not 0) of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            // Send over homing sequence theta for motor 1
            //homing function verified; \
              works for both when theta_f is greater than or smaller than theta_i 
            theta_hom = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
            cout << theta_hom << std::fixed << std::setprecision(3) << ";";  //copy paste this output into cmd.txt

            // Formatting data to send over COM port
            int rounded_theta = theta_hom * 1000;  //*1000 to convert to 3 dp
            itoa(rounded_theta, buf, 10);          //itoa converts int to char array
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);          //writes to COM port


            // ===== Send 2nd hip motor's ID ===== //
            itoa(id[2], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            // Send over homing sequence theta for motor 2
            theta_hom = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
            cout << theta_hom << std::fixed << std::setprecision(3) << ";";

            // Formatting data to send over COM port
            rounded_theta = theta_hom * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);


            // ===== Reading data ===== //
            // !!! NOT PROPERLY IMPLEMENTED YET !!! //

            // Receiving strings
            //readPort.read(enc_str, BYTES_IN);
            //enc_data1[i1] = atof(enc_str);
            //readPort.read(enc_str, BYTES_IN);
            //enc_data2[i2] = atof(enc_str);

            // Receiving float as binary data
            //readPort.readFloat(enc);
            //enc_data1[i1] = enc;
            //readPort.readFloat(enc);
            //enc_data2[i2] = enc;

            //i1++;
            //i2++;


            // Update variables
            /*
             Explanation for corrected_t: t_prev = t_now will work too, but we 
             found tht 'if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms)'
             sometimes returns true even when it has not been 'dt_ms' ms yet. 
             This means the difference between t_now and t_prev is not exactly
             dt_ms, and when looping for thousands of times, this will add up
             to a large timing discrepancy. Hence, t_prev is set to
             'corrected_t + milliseconds(dt*step)' so that even if the if 
             statement triggered before it has been 'dt_ms' ms, the timing error 
             will exist for that time step only. The timing for the next step is
             calculated again using the corrected_t variable as shown in the
             following line, hence preventing timing error accumulation. 
            */
            t_prev = corrected_t + milliseconds(dt*step);
            step++;
        }
    }  //end of homing sequence


    // ==========  Idle before walking  ========== //
    /*
     This section is to keep the motor idle for a few seconds after homing,
     before starting its 'desired trajectory' movement. 
     
     NEEDS FIXING: Prof does not want the program to idle, but instead wants it
     to keep spamming the same motor position command so that it stays at the 
     same position. The idea is to get the motor position feedback even while
     it's idle. (Recall: a feedback msg is sent back for every cmd msg sent.)
     If program is idled instead, then we will have no feedback data while the 
     motor is idling. 
    */

    //std::this_thread::sleep_for(seconds(2));  //this will simple pause the program


    // ========== Motors following the desired trajectory ========== //
    /*
     You can modify the trajectory equation in this section. Remember to also
     update the theta_f_right and theta_f_left values used in the homing
     sequence if you're using a different trajectory. 
     
     Variables:
     - int run_time = how long do you want the trajectory to run?
     - auto running_for = how long has it been since t_pre_loop? 
    */

    // Setting up timing variables for 'desired trajectory' movement
    t_pre_loop = steady_clock::now(); 
    t_prev = t_pre_loop;  //again, not equal in terms of context
    t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000;  //in ms
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop);
    step = 0;  //reset value of step


    // Calculating trajectory theta and sending the commands
    corrected_t = steady_clock::now();
    while(running_for <= run_time_ms) {
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev);
        if(elapsed >= dt_ms) {
            //dt is int but t is still ends up a double, verified
            double t = (dt*pow(10,-3)) * step;
            
            // ===== Send 1st hip motor's ID ===== //
            itoa(id[1], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            // Desired trajectory for 1st motor (change as you see fit)
            theta_d_right = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            cout << theta_d_right << std::fixed << std::setprecision(3) << ";";  // DEBUG

            // Formatting data to send over COM port
            int rounded_theta = theta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            

            // ===== Send 2nd hip motor's ID ===== //
            itoa(id[2], buf, 10);
            id_str_n = buf + newline;
            writePort.write(id_str_n);


            // Desired trajectory for 2nd motor (change as you see fit)
            theta_d_left = (M_PI/180) * (a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) + 
                a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) + 
                a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
                a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);
            
            cout << theta_d_left << std::fixed << std::setprecision(3) << ";";  // DEBUG


            // Formatting data to send over COM port
            rounded_theta = theta_d_left * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);


            // Update variables
            t_prev = corrected_t + milliseconds(dt*step);
            step++;
        }

        t_now = steady_clock::now();
        running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop);
    
    }  //end of desired traj loop

    cout << endl << "All commands sent!\n";
}
