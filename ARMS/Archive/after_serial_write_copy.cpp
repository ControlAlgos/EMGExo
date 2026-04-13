// CMake version for running the main PC executable

// References:
//     https://stackoverflow.com/questions/15794422/serial-port-rs-232-connection-in-c
//     https://web.archive.org/web/20180127160838/http://bd.eduweb.hhs.nl/micprg/pdf/serial-win.pdf
//     https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html
//     (Maybe this one too) https://embetronicx.com/tutorials/wireless/esp32/idf/esp32-idf-serial-communication-tutorial/
//     https://stackoverflow.com/questions/8304190/cmake-with-include-and-source-paths-basic-setup




#define _USE_MATH_DEFINES
#include <cmath>
#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <chrono>       //for working w time
#include <thread>       //for sleeping
#include <iostream>     //cout
#include <iomanip>      //setprecision
#include <cstring>
#include <stdlib.h>     //for itoa()
#include "SerialPC.h"

using namespace std::chrono_literals;
using namespace std::chrono;

DWORD dwReading;

/// @brief: Eddie's walking trajectory constants for the hip joint (by Eddie)
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


/// @brief: Left-Right Hip Trajectory adjustments
float traj_period = 2 * M_PI / w;
float phase_shift = 180;
float b = phase_shift/360 * traj_period;  // phase shift in units of sec;
int step;                                 // keeps track of the number of commands sent

char theta_char[] = "";
std::string newline = "\n";
std::string theta_str_n;
std::string id_str_n;
char buf[10];


/// @brief Preprocesses data to be sent over the COM port. Data might need to be sent as an int instead.
/// @param motor_id             ID of the motor
/// @param buffer               A buffer variable to store data
/// @param writing_port         The serial port to which the motor connects to in the form of a class object
/// @param curr_theta_sequence  The current angular position to send to the motor
void send_sequence_to_motor(int motor_id,
                            char *buffer,
                            SerialPC writing_port,
                            float curr_theta_sequence)
{
    /// @note: Formatting motor ID data to send over COM port
    itoa(motor_id, buffer, 10);
    std::string id_str_n = buffer + newline;
    writing_port.write(id_str_n);

    /// @note: Formatting angular position data to send over COM port
    int rounded_theta = curr_theta_sequence * 1000;
    itoa(rounded_theta, buffer, 10);
    theta_str_n = buffer + newline;
    writing_port.write(theta_str_n);

    // For debugging
    std::cout << "Sending command for motor ID: " << id_str_n << std::endl;
    std::cout << "Sending command for angular position value: " << theta_str_n << std::fixed << std::setprecision(3) << std::endl << std::endl;

}

int main() 
{
    // ==========  Definition and setup for writing to COM port  ========== //
    SerialPC writePort;
    char write_port_num[] = "COM3";
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);

    int id[] = {0, 1, 2};  // Array for motor IDs. Note that we don't really use index 0. This is just to make the code readable.


    // ==========  Definition and setup for reading from COM port  ========== //
    /* 
     If this section can be set up properly, it will allow the program to get
     access to the feedback data. As of now, the feedback data is only printed
     to PuTTY via ESP32's 2nd Serial port and is not directly being read by this
     program. Implementing this section slows down the code and that might have
     to do with the program waiting for something to receive in Serial. Will 
     look further into it.
    */

    SerialPC readPort;
    char read_port_num[] = "COM8";
    readPort.setup(read_port_num, baudrate);
    const int BYTES_IN = 6;  //change back to 6 if sending floats as string; 4 if floats as binary
    char enc_str[BYTES_IN];  //if as strings, leave 1 char out for trailing 0: '\0' if initializing it to sth
    //float enc;

    // Storing data
    //const int SIZE = (run_time / dt) + 50;  //will implement dynamic memory alloc later, use fixed sized arrays for now
    //float *enc_data1 = new float[SIZE];
    
    //float enc_data1[1000];
    //float enc_data2[1000];


    /// @brief: Variables for timing (used in homing and also in desired traj)
    int dt = 2;  //in ms
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //var type = 'Duration'
    auto t_pre_loop = steady_clock::now();   //get timepoint before while loop below
    auto t_prev = t_pre_loop;  //they're not equal in terms of context
    auto t_now = t_pre_loop;   //just using the equal sign cuz didnt want to write steady_clock::now() multiple times
    auto corrected_t = steady_clock::now();  //will explain below

    ///////////////////////////////////////////////////////////////////
    ///////////////// Motor homing sequence ///////////////////////////
    ///////////////////////////////////////////////////////////////////
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
    
    /// @brief: Homing function variables
    double set_homing_dur = 5;            //[seconds]
    double period = set_homing_dur * 2;   //[seconds]; homing = half osc only
    double homing_w = 2 * M_PI / period;  //[rad/s]
    
    /// Calculate start and end theta for homing: theta_i and theta_f
    double theta_i = 0;        // [rad]
    float amp_shift_deg = 23;  // shifts vertically by x deg
    double theta_f_right = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg);  // [rad], t=0

    double theta_f_left = (M_PI/180) * (a0 + a1*cos((0-b)*w) + b1*sin((0-b)*w) + 
            a2*cos(2*(0-b)*w) + b2*sin(2*(0-b)*w) + a3*cos(3*(0-b)*w) + b3*sin(3*(0-b)*w) + 
            a4*cos(4*(0-b)*w) + b4*sin(4*(0-b)*w) + a5*cos(5*(0-b)*w) + b5*sin(5*(0-b)*w) +
            a6*cos(6*(0-b)*w) + b6*sin(6*(0-b)*w) - amp_shift_deg);  // [rad], t=0

    /// Theta var that updates every dt
    double theta_home_motor1;  // [rad]
    double theta_home_motor2;
    step = 0;

    /// @brief: Motor homing loop
    // Loops until the num of steps represents tht it's been 'set_homing_dur' secs
    char user_var = 'N';
    std::cout << "Press Y to perform the motor homing sequence: ";
    std::cin >> user_var;
    if (user_var = 'Y')
    {
        while(step <= set_homing_dur / (dt*0.001))
        {
            t_now = steady_clock::now();
            if(duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) 
            {
                double t = step * dt * 0.001;
                /// @brief: Send homing sequence theta for motor ID 1
                ///         Verified for when theta_i < theta_f < theta_i
                theta_home_motor1 = theta_f_right + (theta_i - theta_f_right) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);
                theta_home_motor2 = theta_f_left + (theta_i - theta_f_left) * ((1 + sin(homing_w * t + M_PI / 2.0)) / 2.0);

                send_sequence_to_motor(id[1], buf, writePort, theta_home_motor1);
                // send_sequence_to_motor(id[2], buf, writePort, theta_home_motor1);

                // ===== Reading data ===== //
                // !!! NOT PROPERLY IMPLEMENTED YET !!! //

                /* Receiving strings

                11-5-2023 jramayrat:
                The function readPort.read() depends on ReadFile and in the reference, it's recommended
                that ReadFile reads 1 byte at a time. I'm suggesting that we 
                repeatedly call this function until a newline character is read. On the 
                ESP32 side, I think we should turn the CAN motor data into a string
                and append a newline character to the end of it. 
                
                References:
                    https://cplusplus.com/forum/windows/105318/

                */
                std::cout << "Where am I " << std::endl;
                std::string data_from_port_read = "";
                int i = 0;
                // while (data_from_port_read[data_from_port_read.back()] != '\n')
                while (i < 3)
                {
                    char* single_char_byte_from_port;
                    std::cout << "here " << std::endl;

                    readPort.read(single_char_byte_from_port, sizeof(single_char_byte_from_port));
                    std::cout << "here too" << std::endl;
                    std::cout << "Single char byte from port: " << single_char_byte_from_port << std::endl;

                    // char *byte_converted_to_char = reinterpret_cast<char*>(single_char_byte_from_port);  // This line may not be
                    data_from_port_read += single_char_byte_from_port;
                    i++;
                }
                std::cout << "here too also" << std::endl;
                i = 0;
                // double converted_data_from_port_read = std::stod(data_from_port_read);
                std::cout << "Data from port read using the read func: " << data_from_port_read << std::endl;
                /// @brief: Original way we did port reads
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
    } else
    {
        std::cout << "User did not enter 'Y'. Cancelling program." << std::endl;
        return 0;
    }

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


    ///////////////////////////////////////////////////////////////////////////
    ////////////////// Walking Trajectory /////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    /*
     You can modify the trajectory equation in this section. Remember to also
     update the theta_f_right and theta_f_left values used in the homing
     sequence if you're using a different trajectory. 
     
     Variables:
     - int run_time = how long do you want the trajectory to run?
     - auto running_for = how long has it been since t_pre_loop? 
    */
    float theta_d_right = 0;
    float theta_d_left = 0;

    /// @brief: Setting up timing variables for 'desired trajectory' movement
    t_pre_loop = steady_clock::now(); 
    t_prev = t_pre_loop;  //again, not equal in terms of context
    t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000;  //in ms
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop);
    step = 0;  //reset value of step


    // Calculating trajectory theta and sending the commands
    user_var = 'N';
    std::cout << "Ready to perform walking trajectory." << std::endl;
    std::cout << "Enter 'Y' to run trajectory: ";
    std::cin >> user_var;
    corrected_t = steady_clock::now();
    if (user_var = 'Y')
    {
        while(running_for <= run_time_ms) 
        {
            auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev);
            if (elapsed >= dt_ms) 
            {
                // dt is int but t is still ends up a double, verified
                double t = (dt*pow(10,-3)) * step;

                // Desired trajectory for 1st motor (change as you see fit)
                theta_d_right = (M_PI/180) * (a0 + a1*cos(t*w) + b1*sin(t*w) + 
                    a2*cos(2*t*w) + b2*sin(2*t*w) + a3*cos(3*t*w) + b3*sin(3*t*w) + 
                    a4*cos(4*t*w) + b4*sin(4*t*w) + a5*cos(5*t*w) + b5*sin(5*t*w) +
                    a6*cos(6*t*w) + b6*sin(6*t*w) - amp_shift_deg);

                // Desired trajectory for 2nd motor (change as you see fit)
                theta_d_left = (M_PI/180) * (a0 + a1*cos((t-b)*w) + b1*sin((t-b)*w) + 
                    a2*cos(2*(t-b)*w) + b2*sin(2*(t-b)*w) + a3*cos(3*(t-b)*w) + b3*sin(3*(t-b)*w) + 
                    a4*cos(4*(t-b)*w) + b4*sin(4*(t-b)*w) + a5*cos(5*(t-b)*w) + b5*sin(5*(t-b)*w) +
                    a6*cos(6*(t-b)*w) + b6*sin(6*(t-b)*w) - amp_shift_deg);

                send_sequence_to_motor(id[1], buf, writePort, theta_d_right);
                // send_sequence_to_motor(id[2], buf, writePort, theta_d_left);

                // Update variables
                t_prev = corrected_t + milliseconds(dt*step);
                step++;
            }

            t_now = steady_clock::now();
            running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop);
        
        }
    } else
    {
        std::cout << "User did not enter 'Y'. Canceling program." << std::endl;
        return 0;
    }

    std::cout << "All walking trajectory commands sent." << std::endl;
    std::cout << "Finishing program." << std::endl;
    return 0;
}