/// https://stackoverflow.com/questions/25590539/sending-data-through-serial-port-in-c

#define _USE_MATH_DEFINES

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>    //for itoa()

#include <math.h>
#include <cmath>
#include <chrono>      //for working w time
#include <thread>      //for sleeping
#include <iomanip>     //setprecision
#include <cstring>
#include "C:\Users\Mo Sharifi\Documents\PlatformIO\Projects\Exoskeleton code\serial-rw\serialComm_WriteAndRead\SerialPC.h"  //my header file

#include <iostream>
#include <fstream>
#include <string>


using namespace std::chrono_literals;
using namespace std::chrono;

using std::cin;
using std::cout;
using std::endl;


////////////////////////////////////////////////////////////////////////////////
//////////////// Generating Homing and Walking Trajectory ///////////////////////
/////////////////////////////////////////////////////////////////////////////////

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
float w  = 0.1879 * 10;  //removed *10 that was there for testing
float theta_d_right = 0;
float theta_d_left = 0;

// Left-Right Hip Trajectory adjustments
float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift/360 * traj_period;  //phase shift in units of sec;
float amp_shift_deg = 23;  //20 deg amp_shift
//float amp_shift_rad = amp_shift_deg * M_PI/180;  //convert to rad, not used



int main() 
{
    int dt = 2;    //this is an int; units in ms
    int step;  // keeps track of the number of commands sent
    auto dt_ms = milliseconds(dt);

    cout << "Attempting to read from Putty log file" << endl;

    std::ifstream log_file;
    std::ofstream results_file;
    results_file.open("serial_data_results.csv");
    results_file << "Position value sent to motor, Serial COM port read results, Putty log file read results";

    /// @note: We'd want to have this command open the file location where we set
    ///        Putty to send the data
    log_file.open("C:\\Users\\Mo Sharifi\\Documents\\PlatformIO\\Projects\\Exoskeleton code\\serial-rw\\serialComm_WriteAndRead\\jramayrat_sand\\putty_testing\\putty.txt");

    /// @brief: Setting up serial port for writing
    SerialPC writePort;
    char write_port_num[] = "COM3"; // CP210x USB to UART Bridge (COM10) Option on computer 
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    std::string id_str_n;
    char buf[10];

    /// @brief: Setting up serial port for reading
    SerialPC readPort;
    char read_port_num[] = "COM8";
    readPort.setup(read_port_num, baudrate);
    const int BYTES_IN = 6;  //change back to 6 if sending floats as string; 4 if floats as binary
    char enc_str[BYTES_IN];  //if as strings, leave 1 char out for trailing 0: '\0' if initializing it to sth
    float enc;

    // Storing data
    int run_time = (2*M_PI / w) * 5 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later

    const int SIZE = (run_time / dt) + 50;  //will implement dynamic memory alloc later, use fixed sized arrays for now
    float *enc_data1 = new float[SIZE];
    float *enc_data2 = new float[SIZE];

    // float enc_data1[1000];
    // float enc_data2[1000];

    // Motor IDs
    const int NUM_OF_MOTORS = 2;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 1, 2};  //right, left


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


    /// Homing function's characteristics
    double set_homing_dur = 5;            //[seconds]
    double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
    double homing_w = 2 * M_PI / period;  //[rad/s]


    /// Variables for timing (used in homing and also in desired traj)
    int dt = 2;  //in ms
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //var type = 'Duration'
    auto t_pre_loop = steady_clock::now();   //get timepoint before while loop below
    auto t_prev = t_pre_loop;  //they're not equal in terms of context
    auto t_now = t_pre_loop;   //just using the equal sign cuz didnt want to write steady_clock::now() multiple times
    auto corrected_t = steady_clock::now();  //will explain below


    // /// Send all connected motors' IDs so that they can all be put into listening mode 
    // for (int i = 0; i <= NUM_OF_MOTORS; i++){
    //    // Pre-processing data to be sent over COM port; modify later to send them as an int maybe
    //    itoa(id[i], buf, 10);
    //    id_str_n = buf + newline;
    //    writePort.write(id_str_n);
    //    cout << id_str_n;
    //    //cout << "this\n";
    // }
    

    // Pause program to let all motors enter listening mode and settle down
    // No need to pause since will wait for user input
    //cout << "wait start" << endl;  
    //std::this_thread::sleep_for(seconds(1));
    //cout << "wait done" << endl;


    // Ask user if want to run again
    std::cout << "Start? (y/n): ";
    std::string run_again;
    std::cin >> run_again;

    while(run_again[0] == 'y')
    {    //run_again is a c string, so it's a char array
        step = 0;

        // ========== Motor homing sequence ========== //

        double set_homing_dur = 5;        //[seconds]
        double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
        double homing_w = 2 * M_PI / period;  //[rad/s]

        //double delta_t = dt_ms.count();  // DEBUG
        int dt = 2;    //this is an int; units in ms
        auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable
        auto t_pre_loop = steady_clock::now(); 
        auto t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
        auto t_now = t_pre_loop;
        auto corrected_t = steady_clock::now();
        int i1 = 0;
        int i2 = 0;

        if (log_file.is_open())
        {
          std::cout << "Log file is opened" << std::endl;
          // while (log_file)
          // {
            while(step <= set_homing_dur / (dt*0.001))
            {
              t_now = steady_clock::now();
              //if(1) {  //DEBUG
              if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) 
              {
                double t = step * dt * 0.001;

                // ===== Send first hip motor's ID ===== //
                itoa(id[1], buf, 10);  //use the id at index 1 of id array above
                id_str_n = buf + newline;
                writePort.write(id_str_n);

                // Send theta of homing sequence
                //this eqn is verified; works for both when theta_f is greater than or lower than theta_i 
                theta_hom = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                //cout << theta_hom << std::fixed << std::setprecision(3) << ";";

                // Formatting data
                int rounded_theta = theta_hom * 1000;
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);
                //cout << theta_str_n;

                /// @brief: Send theta of homing sequence
                ///         This equation works when {theta_i < theta_f < theta_i }
                theta_hom = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                //cout << theta_hom << ";";

                // Formatting data
                rounded_theta = theta_hom * 1000;
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

                /// 1st method: Reading from serial port 
                readPort.read(enc_str, BYTES_IN);
                enc_data1[i1] = atof(enc_str);

                /// 2nd method: Reading from log file which hopefully gets updated in time.
                std::string putty_data_line;
                std::getline(log_file, putty_data_line);
                // float putty_data = std::stof(putty_data_line);

                results_file << enc_str << "," << enc_data1[i1] << "," << putty_data_line << std::endl;

                /// Receiving float as binary data
                // readPort.readFloat(enc);
                // enc_data1[i1] = enc;
                // readPort.readFloat(enc);
                // enc_data2[i2] = enc;

                i1++;
                i2++;

                t_prev = corrected_t + milliseconds(dt*step);
                step++;  //del later cuz there's one below
                //cout << step << "\n";  // DEBUG
              } 
            } 
          // }
        }

        // Ask user if want to run again
        std::cout << "Run again? (y/n): ";
        std::cin >> run_again;
        
    }
    log_file.close();
    results_file.close();
}