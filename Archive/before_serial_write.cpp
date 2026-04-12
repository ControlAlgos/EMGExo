/*******************************************************************************
 Before double-motor implementation!
 
 Use this program together with '1before_serial_receive_and_control_onESP.cpp'
 on ESP32.

 This program runs on a computer and generates a new theta value every dt.
 The theta values can be calculated using any given trajectory equation.
 This program homes and commands one motor to follow the desired trajectory. To 
 do this for multiple motors, use 'after_serial_write.cpp' (on computer) and
 '1after_serial_receive_and_control_onESP.cpp' (on ESP32).

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


/*******  Constants  *******/
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
float theta_d = 0;  


// Walking (ankle joint)
//float a0 = 4.011;
//float a1 =5.292;
//float b1 = 3.613;
//float a2 = 8.582;
//float b2 = -5.954;
//float a3 = -0.4755;
//float b3 = 5.918;
//float a4 = 1.688;
//float b4 = -2.118;
//float a5 = -0.03504;
//float b5 = 1.023;
//float a6 = 1.298;
//float b6 = -0.7245;
//float a7 = 0;
//float b7 = 0;
//float w = 0.04052 * 50;
//float theta_d = 0;


// Walking (Knee joint) 
//float a0 = 25.7;
//float a1 = -3.83;
//float b1 = -19.28;
//float a2 = -8.54;
//float b2 = 17.93;
//float a3 = 1.91;
//float b3 = 3.77;
//float a4 = 1.09;
//float b4 = 1.50;
//float a5 = 2.05;
//float a6 = -0.31;
//float b6 = -0.90;
//float b5 = 0.58;
//float w = 0.03886 * 50;
//float theta_d = 0;


// Sit to stand (Hip joint)
//float a0 = 49.59;
//float a1 = 22.01;
//float b1 = 35.11;
//float a2 = -1.308;
//float b2 = -11.48;
//float a3 = 0.281;
//float b3 = -4.053;
//float a4 = -0.07209;
//float b4 = -0.1631;
//float a5 = 0.01506;
//float b5 = 0.4015;
//float a6 = 0;
//float b6 = 0;
//float stand_sit_time = 3;
//float w  = 4.232/stand_sit_time;
//float theta_d = 0; 


/**********  Global Variables  **********/
int step = 0;  // keeps track of the number of commands sent


int main() {
    // ==========  Definition and setup for writing to COM port  ========== //
    SerialPC writePort;
    char write_port_num[] = "COM3";
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);
    char theta_char[] = "";
    std::string newline = "\n", theta_str_n;
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
    //const int BYTES_IN = 9;  //change back to 7 later if needed
    //char enc_str[BYTES_IN];  //leave 1 char out for trailing 0: '\0' if initializing it to sth
    //float enc_data[10000];
    //float enc;


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
     - double theta_f = motor's final position after homing, which is the same 
                as the initial theta of the desired traj (t=0)
     - float amp_shift_deg = shifts the entire traj to match a real person's gait
    */

    // Calculate start and end theta for homing: theta_i and theta_f
    double theta_i = 0;        //[rad]
    float amp_shift_deg = 23;  //shifts vertically by x deg

    // ========== For Hip joint ========== //
    // End homing position for right hip motor
    double theta_f = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg);  //[rad], t=0

    // End homing position for left hip motor
    //double theta_f = (M_PI/180) * (a0 + a1*cos((0-b)*w) + b1*sin((0-b)*w) + 
    //            a2*cos(2*(0-b)*w) + b2*sin(2*(0-b)*w) + a3*cos(3*(0-b)*w) + b3*sin(3*(0-b)*w) + 
    //            a4*cos(4*(0-b)*w) + b4*sin(4*(0-b)*w) + a5*cos(5*(0-b)*w) + b5*sin(5*(0-b)*w) +
    //            a6*cos(6*(0-b)*w) + b6*sin(6*(0-b)*w) - amp_shift_deg);  //[rad], t=0


    // ========== For ankle joint ========== //
    // End homing position for right ankle motor
    //double theta_f = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
    //    a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
    //    a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) + 
    //    a6*cos(6*0*w) + b6*sin(6*0*w) + a7*cos(7*0*w) + b7*sin(7*0*w) -amp_shift_deg);


    // ========== For knee joint ========== //
    // End homing position for right knee motor
    //double theta_f = (M_PI/180)*(a0 + a1*cos(0*w) + b1*sin(0*w) + 
    //                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
    //                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) - amp_shift_deg);


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
    int rounded_theta;
    step = 0;


    // Motor homing loop
    // Loops until the num of steps represents tht it's been 'set_homing_dur' secs
    while(step <= set_homing_dur / (dt*0.001)) {
        t_now = steady_clock::now();
        if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
            double t = step * dt * 0.001;

            // Send over homing sequence theta to motor
            //homing function verified; \
              works for both when theta_f is greater than or smaller than theta_i 
            theta_hom = theta_f + (theta_i - theta_f) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
            cout << theta_hom << std::fixed << std::setprecision(4) << ",";  //copy paste this output into cmd.txt

            // Formatting data to send over COM port
            rounded_theta = theta_hom * 10000;  //*10000 to convert to 4 dp
            itoa(rounded_theta, buf, 10);           //itoa converts int to char array
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);           //writes to COM port


            // ===== Reading data ===== //
            // !!! NOT PROPERLY IMPLEMENTED YET !!! //

            // Receiving strings
            //readPort.read(enc_str, BYTES_IN);
            //enc_data[i] = atof(enc_str);
            //readPort.read(enc_str, BYTES_IN);
            //enc_data2[i2] = atof(enc_str);

            // Receiving float as binary data
            //readPort.readFloat(enc);
            //enc_data[i] = enc;
            ////readPort.readFloat(enc);
            ////enc_data2[i2] = enc;
            
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
            t_prev = corrected_t + milliseconds(dt * step);
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
    int run_time = 12 * 1000;  //in ms
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

            // Right hip
            theta_d = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            // Left hip
            //theta_d = (M_PI/180) * (a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) + 
            //    a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) + 
            //    a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
            //    a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);

            // Right ankle
            //theta_d = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
            //    a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
            //    a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
            //    a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            // Right knee
            //theta_d = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
            //    a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
            //    a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
            //    a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            cout << theta_d << std::fixed << std::setprecision(4) << ",";  // DEBUG

            // Formatting data to send over COM port
            rounded_theta = theta_d * 10000;
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