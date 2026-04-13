/************************************************************************************
 //after double motors!

 This code is originally inspired by
 https://stackoverflow.com/questions/25590539/sending-data-through-serial-port-in-c

 In order to run this program, you need to build it first. At the menu
 bar at the top of VS Code, go to Terminal -> Run Build Task...
 Then click on the option that has 'g++' in it. You will see that a 
 .exe file has been created in its parent folder.

 To run this executable file, go to its parent folder. In the
 directory bar, type in 'cmd' and press Enter. Then, type '.\filename'
 to run the .exe file.

 By: Sai Hein Si Thu   
 Updated: 2023-MAY-06 1850
 Edits: Jose Jaime EP (May 8)
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

using std::cin;
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
float w  = .65; //0.1879 * 10;  //removed *10 that was there for testing
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
    //auto dt_ms = milliseconds(dt);


    // ========== Set up for Serial communication ========== //
    // Write definition and setup
    SerialPC writePort;
    char write_port_num[] = "COM3"; // CP210x USB to UART Bridge (COM3) Option on computer
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    std::string id_str_n;
    char buf[10];

    // Read port definition and setup
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

    // Motor IDs (Four Joints)
    const int NUM_OF_MOTORS = 3;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 2, 3, 4};  //right (hip), left (hip), right (knee), Left(knee) 
    
    // Send all connected motors' IDs so that they can all be put into listening mode 
    //for (int i = 0; i <= NUM_OF_MOTORS; i++){
    //    // Pre-processing data to be sent over COM port; modify later to send them as an int maybe
    //    itoa(id[i], buf, 10);
    //    id_str_n = buf + newline;
    //    writePort.write(id_str_n);
    //    cout << id_str_n;
    //    //cout << "this\n";
    //}
    

    // Pause program to let all motors enter listening mode and settle down
    // No need to pause since will wait for user input
    //cout << "wait start" << endl;  
    //std::this_thread::sleep_for(seconds(1));
    //cout << "wait done" << endl;


    // Ask user if want to run again
    //cout << "Start? (y/n): ";
    //std::string run_again;
    //cin >> run_again;

    //while(run_again[0] == 'y'){    //run_again is a c string, so it's a char array
        step = 0;

        // ========== Motor HOMING sequence ========== // (HIPS)++++++++++++++++++++++++++++++++++
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

        
         // ========== Motor HOMING sequence ========== // (KNEE)
        double ktheta_f_right= (M_PI/180)*(ka0 + ka1*cos(0*w) + kb1*sin(0*w) + 
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
        //int i1 = 0;
        //int i2 = 0;

        while(step <= set_homing_dur / (dt*0.001)) {
            t_now = steady_clock::now();
            //if(1) {  //DEBUG
            if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
                double t = step * dt * 0.001;

// ===== Send 1st hip motor's ID =========================================================== //
                itoa(id[2], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << std::fixed << std::setprecision(3) << ";";

                // Formatting data
                int rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);
                //cout << theta_str_n;

// ===== Send 3rd hip motor's ID ========================================================== //
                itoa(id[4], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = ktheta_f_right + (theta_i - ktheta_f_right) * -((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

// ===== Send 2nd hip motor's ID ========================================================== //
                itoa(id[6], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

// ===== Send 3rd hip motor's ID ========================================================== //
                itoa(id[12], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = ktheta_f_right + (theta_i - ktheta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);


// ===== Send 4th hip motor's ID ========================================================== //
                itoa(id[15], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = ktheta_f_left + (theta_i - ktheta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);


                // ===== Reading data ===== //
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

                t_prev = corrected_t + milliseconds(dt*step);
                step++;  //del later cuz there's one below
                //cout << step << "\n";  // DEBUG
            } //del later
        } //del later

        //cout << "======= THESE ARE DATA FOR 1st MOTOR ID =======\n";
        //for (int i = 0; i < i1; i++) {
        //    cout << enc_data1[i] << ",\n";
        //}
//
        //cout << endl;
//
        //cout << "======= THESE ARE DATA FOR 2nd MOTOR ID =======\n";
        //for (int i = 0; i < i2; i++) {
        //    cout << enc_data2[i] << ",\n";
        //}


        // Ask user if want to run again
        //cout << "Run again? (y/n): ";
        //cin >> run_again;
        
    //}
//} //homing part ends here (del '}' later)

  //big comment STARTS here

            // Reading data
            // DEBUG
            //char temp[BYTES_IN];
            //strcpy(temp, enc_str);
            //cout << temp << "hello\n";  // DEBUG


            //cout << std::fixed << std::setprecision(3) << enc << "\n";  // DEBUG
            

            //printf("%.3f\r\n", enc_str);


    // Idle before walking //
    //auto idle_time = seconds(2);
    //while(duration_cast<seconds>(steady_clock::now() - t_prev) < idle_time){}


    // Setting up timing for walking trajectory //
    t_pre_loop = steady_clock::now(); 
    t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
    t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
    step = 0;  //reset value of step

    //cout << "Sending commands..." << endl;

    // DEBUG
    //double sum = 0;
    //double max_elapsed = 0;
    //double t_elapsed = 0;

    //Calculating and sending commands
    //while(duration_cast<milliseconds>(steady_clock::now() - t_pre_loop) <= run_time_ms) {  //originally duration_cast<seconds>(...)
    corrected_t = steady_clock::now();
    while(running_for <= run_time_ms) {
        //t_now = steady_clock::now();  //get curr time at the end instead
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev); //updated
        //cout << elapsed.count() << endl;  // DEBUG

        //if (duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
        if(elapsed >= dt_ms) {
            double t = (dt*pow(10,-3)) * step;  //dt is int but t is still ends up a double, verified
            //cout << elapsed.count() << "\t" << t << endl;  // DEBUG
            
// ===== Send 1st hip motor's ID =================================================================== //
            itoa(id[2], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_d_right = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
                a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

            cout << theta_d_right << std::fixed << std::setprecision(3) << ";"; // DEBUG

            int rounded_theta = theta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

// ===== Send 3rd hip motor's ID (RIGHT KNEE) =================================================================== //
            itoa(id[4], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            ktheta_d_right = (M_PI/180) * (ka0 + ka1*cos(t*w) + kb1*sin(t*w) + 
                ka2*cos(2*t*w) + kb2*sin(2*t*w) + ka3*cos(3*t*w) + kb3*sin(3*t*w) + 
                ka4*cos(4*t*w) + kb4*sin(4*t*w) + ka5*cos(5*t*w) + kb5*sin(5*t*w) +
                ka6*cos(6*t*w) + kb6*sin(6*t*w) - amp_shift_deg);

            cout << ktheta_d_right << std::fixed << std::setprecision(3) << ";"; // DEBUG

            rounded_theta = ktheta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            

// ===== Send 2nd hip motor's ID ================================================================== //
            itoa(id[6], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            theta_d_left = (-M_PI/180) * (a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) + 
                a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) + 
                a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
                a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);
            
            cout << theta_d_left << std::fixed << std::setprecision(3) << ";";

            rounded_theta = theta_d_left * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

// ===== Send 3rd hip motor's ID (RIGHT KNEE) =================================================================== //
            itoa(id[12], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            ktheta_d_right = (M_PI/180) * (ka0 + ka1*cos(t*w) + kb1*sin(t*w) + 
                ka2*cos(2*t*w) + kb2*sin(2*t*w) + ka3*cos(3*t*w) + kb3*sin(3*t*w) + 
                ka4*cos(4*t*w) + kb4*sin(4*t*w) + ka5*cos(5*t*w) + kb5*sin(5*t*w) +
                ka6*cos(6*t*w) + kb6*sin(6*t*w) - amp_shift_deg);

            cout << ktheta_d_right << std::fixed << std::setprecision(3) << ";"; // DEBUG

            rounded_theta = ktheta_d_right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);
            

// ===== Send 4th hip motor's ID (LEFT KNEE) ================================================================== //
            itoa(id[15], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            ktheta_d_left = -(-M_PI/180) * (ka0 + ka1*cos((t-b)*w) + kb1*sin((t-b)*w) + 
                ka2*cos(2*(t-b)*w) + kb2*sin(2*(t-b)*w) + ka3*cos(3*(t-b)*w) + kb3*sin(3*(t-b)*w) + 
                ka4*cos(4*(t-b)*w) + kb4*sin(4*(t-b)*w) + ka5*cos(5*(t-b)*w) + kb5*sin(5*(t-b)*w) +
                ka6*cos(6*(t-b)*w) + kb6*sin(6*(t-b)*w) - amp_shift_deg);
            
            cout << ktheta_d_left << std::fixed << std::setprecision(3) << ";";

            rounded_theta = ktheta_d_left * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);


            // DEBUG
            //num_bytes = sizeof(theta_char) / sizeof(theta_char[0]);

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
  //big comment ENDS here