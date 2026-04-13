/************************************************************************************
Motor control code to control two motors with adaptive CPG 
Code is based on original by Sai Hein Si Thu  

By: Eric Kwan
Last updated: 2/14/2024
************************************************************************************/

#include <windows.h>
#include <stdio.h>
#include <math.h>
#include <chrono>      //for working w time
#include <thread>      //for sleeping
#include <iostream>    //cout
#include <sstream>
#include <iomanip>     //setprecision
#include <cstring>
#include <stdlib.h>    //for itoa()
#include "SerialPC.h"  //my header file
#include <string>

using namespace std::chrono_literals;
using namespace std::chrono;

using std::cin;
using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////***T-MOTOR CONSTANTS***/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////***HOMING and EDDIE's TRAJECTORY COEFFICIENT AND CONSTANTS***/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
float w  = 1.2;//2.4; //1.2; //0.1879 * 10;  //removed *10 that was there for testing
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
float amp_shift_deg_homing = 40;  //20 deg amp_shift


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////***ACPG COEFFICIENTS, CONSTANTS, I.C., AND VARIABLE INITIALIZATION***/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**Hip Fourier Series Coefficients**/
float a0_h = 10.13;
float a1_h = 21.80;
float a2_h = -5.07;
float a3_h = -0.49;
float a4_h = -0.52;
float a5_h = 0.20;
float a6_h = -0.07;
float a7_h = -0.09;
float a8_h = -0.09;
float b1_h = -10.77;
float b2_h = -2.21;
float b3_h = 1.86;
float b4_h = 0.41;
float b5_h = 0.20;
float b6_h = -0.06;
float b7_h = -0.05;
float b8_h = -0.05;
// Sharifi's CPG Constants for Knee
/**Knee Fourier Series Coefficients**/
float a0_k = 22.44;
float a1_k = -2.93;
float a2_k = -14.32;
float a3_k = 0.005;
float a4_k = -0.38;
float a5_k = 0.36;
float a6_k = 0.20;
float a7_k = -0.01;
float a8_k = 0.03;
float b1_k = -26.48;
float b2_k = 9.81;
float b3_k = 4.44;
float b4_k = 1.87;
float b5_k = 0.59;
float b6_k = -0.15;
float b7_k = -0.08;
float b8_k = -0.07;


/**Reference Trajectory Constants**/
//Frequency Const:
float gamma_w = 22;
float OMEGA = 0.94;
float psi = 0.141;
//Oscillation Amplitude Const:
float gamma_p = 22;
float A_rho = 1;
float A_rho_knee = 1;
float lamda = 0.056;
float k_rho = 1.6;
float rho_thpos = 1.1;
float rho_max = 1.2;
//Equilibrium Position Const:
float gamma_epsil = 22;
float A_epsil = 10.13;
float A_epsil_knee = 23.44;
float beta = 2;
float k_epsil1 = 5;
float k_epsil2 = 5;
float epsil_thpos = 15.13;
float epsil_thneg = 5.13;
float epsil_max = 18.13;
float epsil_min = 2.13;
//Phase Variation Const:
float N_ij = 0;
//Calculating Torque Motor Feedback
float Kt = 0.136; //this is the torque constant in units of Nm/A obtained from t-motors specificaitons for AK80-64
                  //use Kt = 0.198 for AK10-9 motor or Kt = 0.105 for AK80-9 motor

////Initial Conditions
//**Reference Trajectory Initial Conditions**//
//Frequency I.C.
float w_ddot = 0;
float w_dot = 0;
float w_t = OMEGA; 

float w_knee_ddot = 0;
float w_knee_dot = 0;
float w_t_knee = OMEGA;
//Oscillation Amplitude I.C.
float rho_ddot_Left = 0;
float rho_dot_Left = 0;
float rho_Left = A_rho;
float rho_ddot_Right = 0;
float rho_dot_Right = 0;
float rho_Right = A_rho;

float rho_ddot_LeftKnee = 0;
float rho_dot_LeftKnee = 0; 
float rho_LeftKnee = A_rho_knee;
float rho_ddot_RightKnee = 0;
float rho_dot_RightKnee = 0; 
float rho_RightKnee = A_rho_knee;

//Equilibrium Position I.C.
float epsil_ddot_Left = 0;
float epsil_dot_Left = 0;
float epsil_Left = A_epsil; //[deg]
float epsil_ddot_Right = 0;
float epsil_dot_Right = 0;
float epsil_Right = A_epsil; //[deg]

float epsil_ddot_LeftKnee = 0;
float epsil_dot_LeftKnee = 0;
float epsil_LeftKnee = A_epsil_knee;
float epsil_ddot_RightKnee = 0;
float epsil_dot_RightKnee = 0;
float epsil_RightKnee = A_epsil_knee;

//Phase Variation I.C.
float phi_dot_Left = 0;
float phi_Left = 2 + M_PI; //[rad]
float phi_dot_Right = 0;
float phi_Right = 2; //[rad]


float phi_dot_LeftKnee = 0; 
float phi_LeftKnee = 2 + M_PI;
float phi_dot_RightKnee = 0;
float phi_RightKnee = 2;

//**HRI Energy and Torque**//
float T_HRI_Left = 0; // tal hri left
float T_HRI_Right = 0; //tal hri right
float T_HRI_Left_Knee = 0;
float T_HRI_Right_Knee = 0;
float T_mot_Left = 0; //left motor torque
float T_mot_Right = 0; //right motor torque
float T_mot_Left_Knee = 0;
float T_mot_Right_Knee = 0;
float T_mot_Pass = 0; //passive motor torque
float T_mot_Pass_Knee = 0;
float T_hum_act_Left = 0; //left hip human active torque
float T_hum_act_Right = 0; //right hip human active torque
float T_hum_act_Left_Knee = 0;
float T_hum_act_Right_Knee = 0;
float I_Left = 0; //left hip current feedback
float I_Right = 0; //right hip current feedback
float I_Left_Knee = 0;
float I_Right_Knee = 0;
float E_Left = 0; //HRI energy left
float E_Right = 0; //HRI energy right
float E_Left_Knee = 0;
float E_Right_Knee = 0;
float T_Left = 0; //HRI torque left
float T_Right = 0; //HRI torque right
float T_Left_Knee = 0;
float T_Right_Knee = 0;
float q_r_Left = 0; //angular position left
float q_r_dot_Left_fdbk = 0; //angular velocity left
float q_r_Right = 0; //angular position right
float q_r_dot_Right_fdbk = 0; //angular velocity right
float q_r_Right_Knee = 0; //angular position right knee
float q_r_dot_Right_Knee_fdbk = 0;
float q_r_Left_Knee = 0;
float q_r_Right_Knee_fdbk = 0;

////Current & Previous varaibles for locomotion frequency ddot
float curr_w_dot_arg = w_dot;
float curr_w_dot_knee_arg = w_knee_dot;
//initial conditions for 1st loop iteration
float curr_w_arg = w_t;
float curr_w_knee_arg = w_t_knee;

////Current & Previous varaibles for oscillation amplitude ddot
float curr_rho_dot_Left_arg = rho_dot_Left;
float curr_rho_dot_Right_arg = rho_dot_Right;
float curr_rho_dot_Left_Knee_arg = rho_dot_LeftKnee;
float curr_rho_dot_Right_Knee_arg = rho_dot_RightKnee;
//initial conditions for 1st loop iteration
float curr_rho_Left_arg = rho_Left;
float curr_rho_Right_arg = rho_Right;
float curr_rho_Left_Knee_arg = rho_LeftKnee;
float curr_rho_Right_Knee_arg = rho_RightKnee;
////Current & Previous varaibles for equilibrium ddot
// initial conditions for the 1st loop iteration:
float curr_epsil_dot_Left_arg = epsil_dot_Left;
float curr_epsil_dot_Right_arg = epsil_dot_Right;
float curr_epsil_dot_Left_Knee_arg = epsil_dot_LeftKnee;
float curr_epsil_dot_Right_Knee_arg = epsil_dot_RightKnee;
//initial conditions for 1st loop iteration
float curr_epsil_Left_arg = epsil_Left; //@note: We don't use "curr_equilibrium_pos_ddot_arg" because this is computed within the for loop.???
float curr_epsil_Right_arg = epsil_Right;
float curr_epsil_Left_Knee_arg = epsil_LeftKnee;
float curr_epsil_Right_Knee_arg = epsil_RightKnee;
  
//***Integration 1st Derivation Variables***//
////Current and Previous variables for phase variation dot
float curr_phi_Left_arg = phi_Left;
float curr_phi_Right_arg = phi_Right;
float curr_phi_Left_Knee_arg = phi_LeftKnee;
float curr_phi_Right_Knee_arg = phi_RightKnee;

//HRI Energy and Torque Integration Variables
float curr_T_Left_arg = 0;
float curr_E_Left_arg = 0;
float curr_T_Right_arg = 0;
float curr_E_Right_arg = 0;
float curr_T_Left_Knee_arg = 0;
float curr_E_Left_Knee_arg = 0;
float curr_T_Right_Knee_arg = 0;
float curr_E_Right_Knee_arg = 0;

std::string data;
char buf[10];

// Global declaration for parsing and converting
float velocity_LeftHip = 0, current_LeftHip = 0, position_LeftHip = 0, velocity_RightHip = 0, current_RightHip = 0, position_RightHip = 0, velocity_LeftKnee = 0, current_LeftKnee = 0, position_LeftKnee = 0, velocity_RightKnee = 0, current_RightKnee = 0, position_RightKnee = 0;
float accel_x1 = 0, accel_y1 = 0, accel_z1 = 0, accel_x2 = 0, accel_y2 = 0, accel_z2 = 0, accel_x3 = 0, accel_y3 = 0, accel_z3 = 0, gyro_x1 = 0, gyro_y1 = 0, gyro_z1 = 0, gyro_x2 = 0, gyro_y2 = 0, gyro_z2 = 0, gyro_x3 = 0, gyro_y3 = 0, gyro_z3 = 0;

//Limiting Current Values "DEAD ZONE"
float CurrentDifference_LeftHip = 0;
float LimitTorque_LeftHip = 0;
float CurrentDifference_RightHip = 0;
float LimitTorque_RightHip = 0;
// float CurrentDifference_LeftKnee = 0;
// float LimitTorque_LeftKnee = 0;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////***VOID FUNCTIONS FOR MULTITHREADING***/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterCalculation_loop(double t, int dt){

  //Left Hip
  w_ddot = gamma_w*((gamma_w/4)*(OMEGA + psi*(E_Left+E_Right) - w_t) - w_dot);
  w_dot = curr_w_dot_arg + w_ddot*t;
  w_t = curr_w_arg + w_dot*t;
  curr_w_dot_arg = w_dot; //To update the current w_dot integration value
  curr_w_arg = w_t; //To update the current w integration value

  rho_ddot_Left = gamma_p*((gamma_p/4)*(A_rho + lamda*E_Left - rho_Left) - rho_dot_Left);  //note the threshold need to be added!!
  rho_dot_Left = curr_rho_dot_Left_arg + rho_ddot_Left*t;
  rho_Left = curr_rho_Left_arg + rho_dot_Left*t;
  curr_rho_dot_Left_arg = rho_dot_Left; //To update the current rho_dot integration value
  curr_rho_Left_arg = rho_Left; //To update the current rho integration value

  epsil_ddot_Left = gamma_epsil*((gamma_epsil/4)*(A_epsil + beta*T_HRI_Left - epsil_Left) - epsil_dot_Left); //note the threshold need to be added!!
  epsil_dot_Left = curr_epsil_dot_Left_arg + epsil_ddot_Left*t;
  epsil_Left = curr_epsil_Left_arg + epsil_dot_Left*t;
  curr_epsil_dot_Left_arg = epsil_dot_Left; //To update the current epsil_dot integration value
  curr_epsil_Left_arg = epsil_Left; //To update the current epsil integration value
      
  phi_dot_Left = w_t; //+ N_ij*sin(prev_int_phi_Left-curr_phi_left-(prev_int_phi_Left-curr_phi_left)); // do not need the other half for now.
  phi_Left = curr_phi_Left_arg + (phi_dot_Left*(dt)/1000); // from simulation, the last part of the function must equate to 0.00188 for a sampling time of 2ms or else the trajectory will not work. phi_Left = curr_phi_Left_arg + 0.00188; 
  curr_phi_Left_arg = phi_Left;

  //Right Hip
  rho_ddot_Right = gamma_p*((gamma_p/4)*(A_rho + lamda*E_Right - rho_Right) - rho_dot_Right);  //note the threshold need to be added!!
  rho_dot_Right= curr_rho_dot_Right_arg + rho_ddot_Right*t;
  rho_Right = curr_rho_Right_arg + rho_dot_Right*t;
  curr_rho_dot_Right_arg = rho_dot_Right; //To update the current rho_dot integration value
  curr_rho_Right_arg = rho_Right; //To update the current rho integration value

  epsil_ddot_Right = gamma_epsil*((gamma_epsil/4)*(A_epsil + beta*T_HRI_Right - epsil_Right) - epsil_dot_Right); //note the threshold need to be added!!
  epsil_dot_Right = curr_epsil_dot_Right_arg + epsil_ddot_Right*t;
  epsil_Right = curr_epsil_Right_arg + epsil_dot_Right*t;
  curr_epsil_dot_Right_arg = epsil_dot_Right; //To update the current epsil_dot integration value
  curr_epsil_Right_arg = epsil_Right; //To update the current epsil integration value
      
  phi_dot_Right = w_t; //+ N_ij*sin(prev_int_phi_Left-curr_phi_left-(prev_int_phi_Left-curr_phi_left)); // do not need the other half for now.
  phi_Right = curr_phi_Right_arg + (phi_dot_Right*(dt)/1000); // from simulation, the last part of the function must equate to 0.00188 for a sampling time of 2ms or else the trajectory will not work. phi_Left = curr_phi_Left_arg + 0.00188; 
  curr_phi_Right_arg = phi_Right; //phase variation is in degrees but overall the trajectory need to be in radians

  //Left Knee
  w_knee_ddot = gamma_w*((gamma_w/4)*(OMEGA + psi*(E_Left_Knee+E_Right_Knee) - w_t_knee) - w_knee_dot); //note: energy added together here as summation of all joints
  w_knee_dot = curr_w_dot_knee_arg + w_knee_ddot*t;
  w_t_knee = curr_w_knee_arg + w_knee_dot*t;
  curr_w_dot_knee_arg = w_knee_dot; //To update the current w_dot integration value
  curr_w_knee_arg = w_t_knee; //To update the current w integration value

  rho_ddot_LeftKnee = gamma_p*((gamma_p/4)*(A_rho_knee + lamda*E_Left_Knee - rho_LeftKnee) - rho_dot_LeftKnee);  //note the threshold need to be added!!
  rho_dot_LeftKnee = curr_rho_dot_Left_Knee_arg + rho_ddot_LeftKnee*t;
  rho_LeftKnee = curr_rho_Left_Knee_arg + rho_dot_LeftKnee*t;
  curr_rho_dot_Left_Knee_arg = rho_dot_LeftKnee; //To update the current rho_dot integration value
  curr_rho_Left_Knee_arg = rho_LeftKnee; //To update the current rho integration value

  epsil_ddot_LeftKnee = gamma_epsil*((gamma_epsil/4)*(A_epsil_knee + beta*T_HRI_Left_Knee - epsil_LeftKnee) - epsil_dot_LeftKnee); //note the threshold need to be added!!
  epsil_dot_LeftKnee = curr_epsil_dot_Left_Knee_arg + epsil_ddot_LeftKnee*t;
  epsil_LeftKnee = curr_epsil_Left_Knee_arg + epsil_dot_LeftKnee*t;
  curr_epsil_dot_Left_Knee_arg = epsil_dot_LeftKnee; //To update the current epsil_dot integration value
  curr_epsil_Left_Knee_arg = epsil_LeftKnee; //To update the current epsil integration value

  phi_dot_LeftKnee = w_t_knee; //+ N_ij*sin(prev_int_phi_Left-curr_phi_left-(prev_int_phi_Left-curr_phi_left)); // do not need the other half for now.
  phi_LeftKnee = curr_phi_Left_Knee_arg + (phi_dot_LeftKnee*(dt)/1000); // from simulation, the last part of the function must equate to 0.00188 for a sampling time of 2ms or else the trajectory will not work. phi_Left = curr_phi_Left_arg + 0.00188; 
  curr_phi_Left_Knee_arg = phi_LeftKnee;

  //Right Knee
  rho_ddot_RightKnee = gamma_p*((gamma_p/4)*(A_rho_knee + lamda*E_Right_Knee - rho_RightKnee) - rho_dot_RightKnee);  //note the threshold need to be added!!
  rho_dot_RightKnee = curr_rho_dot_Right_Knee_arg + rho_ddot_RightKnee*t;
  rho_RightKnee = curr_rho_Right_Knee_arg + rho_dot_RightKnee*t;
  curr_rho_dot_Right_Knee_arg = rho_dot_RightKnee; //To update the current rho_dot integration value
  curr_rho_Right_Knee_arg = rho_RightKnee; //To update the current rho integration value

  epsil_ddot_RightKnee = gamma_epsil*((gamma_epsil/4)*(A_epsil_knee + beta*T_HRI_Right_Knee - epsil_RightKnee) - epsil_dot_RightKnee); //note the threshold need to be added!!
  epsil_dot_RightKnee = curr_epsil_dot_Right_Knee_arg + epsil_ddot_RightKnee*t;
  epsil_RightKnee = curr_epsil_Right_Knee_arg + epsil_dot_RightKnee*t;
  curr_epsil_dot_Right_Knee_arg = epsil_dot_RightKnee; //To update the current epsil_dot integration value
  curr_epsil_Right_Knee_arg = epsil_RightKnee; //To update the current epsil integration value

  phi_dot_RightKnee = w_t_knee; //+ N_ij*sin(prev_int_phi_Left-curr_phi_left-(prev_int_phi_Left-curr_phi_left)); // do not need the other half for now.
  phi_RightKnee = curr_phi_Right_Knee_arg + (phi_dot_RightKnee*(dt)/1000); // from simulation, the last part of the function must equate to 0.00188 for a sampling time of 2ms or else the trajectory will not work. phi_Left = curr_phi_Left_arg + 0.00188; 
  curr_phi_Right_Knee_arg = phi_RightKnee;
}


// void leftHipTrajCalculation_loop(){
//  q_r_Left = (M_PI/180)*(epsil_Left + rho_Left * (a0_h + a1_h*cos(1*phi_Left) + b1_h*sin(1*phi_Left) + a2_h*cos(2*phi_Left) + 
//             b2_h*sin(2*phi_Left) + a3_h*cos(3*phi_Left) + b3_h*sin(3*phi_Left) + a4_h*cos(4*phi_Left) + b4_h*sin(4*phi_Left) + a5_h*cos(5*phi_Left) + 
//             b5_h*sin(5*phi_Left) + a6_h*cos(6*phi_Left) + b6_h*sin(6*phi_Left) + a7_h*cos(7*phi_Left)+ b7_h*sin(7*phi_Left) + a8_h*cos(8*phi_Left)+ b8_h*sin(8*phi_Left))); 
// }

// void rightHipTrajCalculation_loop(){
//   q_r_Right = (M_PI/180)*(epsil_Right + rho_Right * (a0_h + a1_h*cos(1*(phi_Right)) + b1_h*sin(1*(phi_Right)) + a2_h*cos(2*(phi_Right)) + 
//             b2_h*sin(2*(phi_Right)) + a3_h*cos(3*(phi_Right)) + b3_h*sin(3*(phi_Right)) + a4_h*cos(4*(phi_Right)) + b4_h*sin(4*(phi_Right)) + a5_h*cos(5*(phi_Right)) + 
//             b5_h*sin(5*(phi_Right)) + a6_h*cos(6*(phi_Right)) + b6_h*sin(6*(phi_Right)) + a7_h*cos(7*(phi_Right)) + b7_h*sin(7*(phi_Right)) + a8_h*cos(8*(phi_Right)) + b8_h*sin(8*(phi_Right))-amp_shift_deg)); 
// }

// void leftKneeTrajCalculation_loop(){
//   q_r_Left_Knee = (M_PI/180)*(epsil_LeftKnee + rho_LeftKnee * (a0_k + a1_k*cos(1*phi_LeftKnee) + b1_k*sin(1*phi_LeftKnee) + a2_k*cos(2*phi_LeftKnee) + 
//             b2_k*sin(2*phi_LeftKnee) + a3_k*cos(3*phi_LeftKnee) + b3_k*sin(3*phi_LeftKnee) + a4_k*cos(4*phi_LeftKnee) + b4_k*sin(4*phi_LeftKnee) + a5_k*cos(5*phi_LeftKnee) + 
//             b5_k*sin(5*phi_LeftKnee) + a6_k*cos(6*phi_LeftKnee) + b6_k*sin(6*phi_LeftKnee) + a7_k*cos(7*phi_LeftKnee)+ b7_k*sin(7*phi_LeftKnee) + a8_k*cos(8*phi_LeftKnee)+ b8_k*sin(8*phi_LeftKnee))); 
// }

// void rightKneeTrajCalculation_loop(){
//   q_r_Right_Knee = (M_PI/180)*(epsil_RightKnee + rho_RightKnee * (a0_k + a1_k*cos(1*(phi_RightKnee)) + b1_k*sin(1*(phi_RightKnee)) + a2_k*cos(2*(phi_RightKnee)) + 
//             b2_k*sin(2*(phi_RightKnee)) + a3_k*cos(3*(phi_RightKnee)) + b3_k*sin(3*(phi_RightKnee)) + a4_k*cos(4*(phi_RightKnee)) + b4_k*sin(4*(phi_RightKnee)) + a5_k*cos(5*(phi_RightKnee)) + 
//             b5_k*sin(5*(phi_RightKnee)) + a6_k*cos(6*(phi_RightKnee)) + b6_k*sin(6*(phi_RightKnee)) + a7_k*cos(7*(phi_RightKnee)) + b7_k*sin(7*(phi_RightKnee)) + a8_k*cos(8*(phi_RightKnee)) + b8_k*sin(8*(phi_RightKnee))-amp_shift_deg));   
// }


void decoding_loop(){


SerialPC readPort;
//*******Read Serial Port for feedback*******//
int bytesRead = readPort.read(buf, sizeof(buf));
  if (bytesRead > 0) {
    // Convert the received data to an integer
    buf[bytesRead] = '\0'; // Ensure null-termination
    //int receivedValue = std::stoi(buf);
    std::string receivedValue(buf);

    data = receivedValue;
  }
    
    std::istringstream iss (data);
    std::string token;
                      
    while (iss >> token) {
      if (token == "P1") {
        iss >> position_LeftHip;
      } 
      else if (token == "C1") 
      {
        iss >> current_LeftHip;
      } 
      else if (token == "V1") {
        iss >> velocity_LeftHip;  
      } 
      else if (token == "P2") {
        iss >> position_RightHip;
      } 
      else if (token == "C2") {
        iss >> current_RightHip;
      }  
      else if (token == "V2") {
        iss >> velocity_RightHip;
      }  
      else if (token == "P3") {
        iss >> position_LeftKnee;
      } 
      else if (token == "C3") {
        iss >> current_LeftKnee;
      }  
      else if (token == "V3") {
        iss >> velocity_LeftKnee;
      } 
      else if (token == "P4") {
        iss >> position_RightKnee;
      } 
      else if (token == "C4") {
        iss >> current_RightKnee;
      }  
      else if (token == "V4") {
        iss >> velocity_RightKnee;
      }
      else if (token == "theta_IMUHip_ddot"){
        //assign to 
      }
      else if (token == "Tal_L"){
        iss >> T_HRI_Left;
      }
      else if (token == "Tal_R"){
        iss >> T_HRI_Right;
      }
}

// void HRICalculation_loop(float t, float dt){
//     /////////////////////////////////////Limiting Current Values "DEAD ZONE"//////////////////////////////////////////////////////////////
//     float I_Pos = 4; //[mA] setting upper bound value for current based on experiemental data w/o human active torque
//     float I_Neg = -4; //[mA] setting lower bound value for current based on experimental data w/o human active torque

//     if (current_LeftHip > I_Pos){
//       CurrentDifference_LeftHip = (current_LeftHip - I_Pos)/1000;
//       LimitTorque_LeftHip = Kt*CurrentDifference_LeftHip;
//       T_HRI_Left = LimitTorque_LeftHip;
//     }
//     else if (current_LeftHip < I_Neg){
//         CurrentDifference_RightHip = (current_LeftHip - I_Neg)/1000; //in this case current_LeftHip would be negative value thus - the I_Neg will give the difference however, because its negative overall, we need negative sign at the end
//         LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
//         T_HRI_Left = -LimitTorque_RightHip;
//     }

//     if (current_RightHip > I_Pos){
//       CurrentDifference_RightHip = (current_RightHip - I_Pos)/1000;
//       LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
//       T_HRI_Left = LimitTorque_RightHip;
//     }
//     else if (current_LeftHip < I_Neg){
//         CurrentDifference_RightHip = (current_LeftHip - I_Neg)/1000; //in this case current_LeftHip would be negative value thus - the I_Neg will give the difference however, because its negative overall, we need negative sign at the end
//         LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
//         T_HRI_Left = -LimitTorque_RightHip;
//     }
//       // else{
//       //     T_HRI_Left = 0;
//       // }

//       //***HRI TORQUE AND ENERGY***//
//       // T_Left = curr_T_Left_arg + (T_HRI_Left*t);
//       // curr_T_Left_arg = T_Left;
//       // E_Left = curr_E_Left_arg + (T_HRI_Left*velocity_LeftHip)*t;
//       // curr_E_Left_arg = E_Left;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////***MAIN FUNCTION***/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main() {
    int step;  // keeps track of the number of commands sent
    //auto dt_ms = milliseconds(dt);
    //auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable
    // ========== Set up for Serial communication ========== //
    //Write definition and setup
    SerialPC writePort;
    char write_port_num[] = "COM3"; // CP210x USB to UART Bridge (COM3) Option on computer
    int baudrate = 115200;
    writePort.setup(write_port_num, baudrate);  //set to COM3; change COM port inside SerialPC::setup() method
    char theta_char[] = "";
    std::string newline = "\n";
    std::string theta_str_n;
    std::string id_str_n;
   

    // Read port definition and setup
    SerialPC readPort;
    char read_port_num[] = "COM5";
    readPort.setup(read_port_num, baudrate);


    // Motor IDs (Four Joints)
    const int NUM_OF_MOTORS = 2;  //just the two hip motors for now
    int id[NUM_OF_MOTORS + 1] = {NUM_OF_MOTORS, 1, 2};//{NUM_OF_MOTORS, 1, 2, 3, 4};  //right (hip), left (hip), right (knee), Left(knee) 
    
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
    // cout << "wait start" << endl;  
    // std::this_thread::sleep_for(seconds(1));
    // cout << "wait done" << endl;


    // Ask user if want to run again
    // cout << "Start? (y/n): ";
    // std::string run_again;
    // cin >> run_again;

  

    //while(run_again[0] == 'y'){    //run_again is a c string, so it's a char array
        step = 0;

        // ========== Motor HOMING sequence ========== // (HIPS)++++++++++++++++++++++++++++++++++
        double theta_hom;                     //[rad]
        double theta_i = 0;                   //[rad]
        double theta_f_right = (M_PI/180) * (a0 + a1*cos(0*w) + b1*sin(0*w) + 
                    a2*cos(2*0*w) + b2*sin(2*0*w) + a3*cos(3*0*w) + b3*sin(3*0*w) + 
                    a4*cos(4*0*w) + b4*sin(4*0*w) + a5*cos(5*0*w) + b5*sin(5*0*w) +
                    a6*cos(6*0*w) + b6*sin(6*0*w) - amp_shift_deg_homing);  //[rad]; the multiplier pi/180 does not make sense lol, will fix it later
        
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
        int i1 = 0;
        int i2 = 0;

        while(step <= set_homing_dur / (dt*0.001)) {
            t_now = steady_clock::now();
            //if(1) {  //DEBUG
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
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);
                //cout << theta_str_n;

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
                //cout << rounded_theta/1000.0 << std::fixed << std::setprecision(3) << "," << "\n";  // DEBUG, changed it frm endl to \n
                itoa(rounded_theta, buf, 10);
                theta_str_n = buf + newline;
                writePort.write(theta_str_n);

// // ===== Send 3rd hip motor's ID ========================================================== //
                itoa(id[3], buf, 10);  //use the id at index 1 of id array above
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


// // ===== Send 4th hip motor's ID ========================================================== //
                itoa(id[4], buf, 10);  //use the id at index 1 of id array above
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


//                 // ===== Reading data ===== //
//                 // Receiving strings
//                 readPort.read(enc_str, BYTES_IN);
//                 enc_data1[i1] = atof(enc_str);
//                 readPort.read(enc_str, BYTES_IN);
//                 enc_data2[i2] = atof(enc_str);

//                 // Receiving float as binary data
//                 readPort.readFloat(enc);
//                 enc_data1[i1] = enc;
//                 readPort.readFloat(enc);
//                 enc_data2[i2] = enc;

//                 //i1++;
//                 //i2++;

//                 t_prev = corrected_t + milliseconds(dt*step);
//                 step++;  //del later cuz there's one below
//                 //cout << step << "\n";  // DEBUG
//             } //del later
//         } //del later

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
    int dt = 10;
    auto dt_ms = std::chrono::duration<float, std::milli>(dt);  //this is a type 'duration' variable

    // Setting up timing for walking trajectory //
    auto t_pre_loop = steady_clock::now(); 
    auto t_prev = t_pre_loop; //they're not equal in terms of context; just using the equal sign becuz didnt want to write steady_clock::now() multiple times
    auto t_now = t_pre_loop;
    int run_time = (2*M_PI / w) * 5 * 1000; // in milliseconds; with 3 dp so *1000; remove the *6 later
    auto run_time_ms = std::chrono::duration<float, std::milli>(run_time);
    auto running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated
    step = 0;  //reset value of step
    float amp_shift_deg = 0; 

   

    //Calculating and sending commands
    auto corrected_t = steady_clock::now();
    while(running_for <= run_time_ms) {
        auto elapsed = std::chrono::duration<float, std::milli>(t_now - t_prev); //updated

        //if (duration_cast<milliseconds>(t_now - t_prev) >= dt_ms) {
        if(elapsed >= dt_ms) {
            double t = (dt*pow(10,-3)) * step;  //dt is int but t is still ends up a double, verified

            //*******Read Serial Port for feedback*******//
            int bytesRead = readPort.read(buf, sizeof(buf));
            if (bytesRead > 0) {
                // Convert the received data to an integer
                buf[bytesRead] = '\0'; // Ensure null-termination
                std::string receivedValue(buf);
                data = receivedValue;
            }

            /////////////////////////////////////////////////////////THREADING INITIALIZATION AND EXECUTION/////////////////////////////////////////////////
            std::thread ParamThread(ParameterCalculation_loop,t, dt); // Construct new thread and run it. Does not block execution
            std::thread DecodeThread(decoding_loop); // Construct new decoding thread and run it for feedback
            // // std::thread HRIThread(HRICalculation_loop, t, dt); // Construct new thread and run it for HRI Torque and Energy Calculation
            DecodeThread.join(); //join decode threads to wait for them to finish
            ParamThread.join();//join the Parameter calculation threads to wait for them to finish
            // HRIThread.join(); //join the HRI calculation thread
            
            //NOTE: has to be in the while loop otherwise will not work

            if (T_HRI_Left > 0){
              w_t = 0.94+T_HRI_Left;
            }
            else if (T_HRI_Left < 0){
              w_t = 0.94 - T_HRI_Left;
            }
            else{
              w_t = 0.94;
            }
           
            //note, it is not necessary to explicityly "close" threads in teh same way would close file. program will
            //automatically terminate. Just need to insure to join all threads before program exits. If do not, it can lead to undefined behavior
            //or resource leaks. Joining a thread means that the program will wait for that thread to finish its execution before proceeding.
            //"join()" ensures that all threads have finished executing before the program terminates

            /**********************************************************************************************************************
            ***parsing received data. Note the extraction pointer '>>' takes care of converting
            the string to a float during the extraction process. The erase function is used to remove the
            trainling semicolon from the tokens before parsing the values.
            **********************************************************************************************************************/
                       
            //*******Read Serial Port for feedback*******//
            // int bytesRead = readPort.read(buf, sizeof(buf));
            // if (bytesRead > 0) {
            //      // Convert the received data to an integer
            //      buf[bytesRead] = '\0'; // Ensure null-termination
            //      //int receivedValue = std::stoi(buf);
            //      std::string receivedValue(buf);

            //      //Print the received integer
            //          //std::cout << "Received Value: " << receivedValue << std::endl;
            //          //std::cout << receivedValue;
            //      data = receivedValue;
            //  }
        
            // std::istringstream iss (data);
            // std::string token;
                      
            // while (iss >> token) {

            //     if (token == "P1") {
            //         iss >> position_LeftHip;
            //     } else if (token == "C1") {
            //         iss >> current_LeftHip;
            //     } else if (token == "V1") {
            //         iss >> velocity_LeftHip;  
            //     } else if (token == "P2") {
            //         iss >> position_RightHip;
            //     } else if (token == "C2") {
            //         iss >> current_RightHip;
            //     }  else if (token == "V2") {
            //         iss >> velocity_RightHip;
            //     }  else if (token == "P3") {
            //         iss >> position_LeftKnee;
            //     } else if (token == "C3") {
            //         iss >> current_LeftKnee;
            //     }  else if (token == "V3") {
            //         iss >> velocity_LeftKnee;
            //     } else if (token == "P4") {
            //         iss >> position_RightKnee;
            //     } else if (token == "C4") {
            //         iss >> current_RightKnee;
            //     }  else if (token == "V4") {
            //         iss >> velocity_RightKnee;
            //     }

            //**********parsing for IMU data***********//
            //     else if (token == "x1") {
            //         iss >> accel_x1;
            //     } else if (token == "y1") {
            //         iss >> accel_y1;
            //     } else if (token == "z1") {
            //         iss >> accel_z1;  
            //     } else if (token == "x2") {
            //         iss >> accel_x2;
            //     } else if (token == "y2") {
            //         iss >> accel_y2;
            //     }  else if (token == "z2") {
            //         iss >> accel_y3;
            //     }  else if (token == "x3") {
            //         iss >> accel_x3;
            //     } else if (token == "y3") {
            //         iss >> accel_y3;
            //     }  else if (token == "z3") {
            //         iss >> accel_z3;
            //     }
            //     else if (token == "") {
            //         iss >> gyro_x1;
            //     } else if (token == "") {
            //         iss >> gyro_y1;
            //     } else if (token == "") {
            //         iss >> gyro_z1;  
            //     } else if (token == "") {
            //         iss >> gyro_x2;
            //     } else if (token == "") {
            //         iss >> gyro_y2;
            //     }  else if (token == "") {
            //         iss >> gyro_y3;
            //     }  else if (token == "") {
            //         iss >> gyro_x3;
            //     } else if (token == "") {
            //         iss >> gyro_y3;
            //     }  else if (token == "") {
            //         iss >> gyro_z3;
            //     }
            // }
    // float I_Pos = 4; //[mA] setting upper bound value for current based on experiemental data w/o human active torque
    // float I_Neg = -4; //[mA] setting lower bound value for current based on experimental data w/o human active torque

    // if (current_LeftHip > I_Pos){
    //   CurrentDifference_LeftHip = (current_LeftHip - I_Pos)/1000;
    //   LimitTorque_LeftHip = Kt*CurrentDifference_LeftHip;
    //   // T_HRI_Left = LimitTorque_LeftHip;
    // }
    // else if (current_LeftHip < I_Neg){
    //     CurrentDifference_RightHip = (current_LeftHip - I_Neg)/1000; //in this case current_LeftHip would be negative value thus - the I_Neg will give the difference however, because its negative overall, we need negative sign at the end
    //     LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
    //     // T_HRI_Left = -LimitTorque_RightHip;
    // }

    // if (current_RightHip > I_Pos){
    //   CurrentDifference_RightHip = (current_RightHip - I_Pos)/1000;
    //   LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
    //   // T_HRI_Left = LimitTorque_RightHip;
    // }
    // else if (current_LeftHip < I_Neg){
    //     CurrentDifference_RightHip = (current_LeftHip - I_Neg)/1000; //in this case current_LeftHip would be negative value thus - the I_Neg will give the difference however, because its negative overall, we need negative sign at the end
    //     LimitTorque_RightHip = Kt*CurrentDifference_RightHip;
        // T_HRI_Left = -LimitTorque_RightHip;
    // }

    // LimitTorque_LeftHip = Kt*(current_LeftHip/1000);
    // LimitTorque_RightHip = Kt*(current_RightHip/1000);
        
            //debugging parse method
            //cout << "Pos1: " << position_L << " Cur1: " << current_L << " Vel1: " << velocity_L << std::fixed << std::setprecision(3) << ";" << std::endl;
            //cout << "Pos2: " << position_R << " Cur2: " << current_R << " Vel2: " << velocity_R << std::fixed << std::setprecision(3) << ";" << std::endl;
            // cout << position_L  //feedback position with torque update

            // cout << position_LeftHip << std::fixed << std::setprecision(3) << ";";// DEBUG
            // cout << position_RightHip << std::fixed << std::setprecision(3) << ";";// DEBUG

            // cout << current_LeftHip << std::fixed << std::setprecision(3) << "; " << current_RightHip  << std::fixed << std::setprecision(3) << ";";
            // cout << LimitTorque_LeftHip << std::fixed << std::setprecision(6) << "; " << LimitTorque_RightHip << std::fixed << std::setprecision(6) << ";";

            ////////////////////////////////////ACCESSING SYS IDENTIFICATION AND OBTAIN PASSIVE MOTOR TORQUE//////////////////////////////////////////////////







            ////////////////////////////////////HIR TORQUE AND ENERGY/////////////////////////////////////////////////////////////
            // I_Left = (current_LeftHip/1000);
            // q_r_dot_Left_fdbk = velocity_LeftHip;

            // I_Right = (current_R/1000);
            // q_r_dot_Right_fdbk = velocity_R;

            //**debug only:
            // cout << I_Left << std::fixed << std::setprecision(3) << ";";
            //cout << "L_Hip_Vel_Fdbk: " << q_r_dot_Left_fdbk;

            // cout << I_Right << std::fixed << std::setprecision(3) << ";";
            //cout <<  "R_Hip_Vel_Fdbk: " << q_r_dot_Right_fdbk;

            //cout << I_Left << "\n";
            // cout << q_r_dot_Left_fdbk << std::fixed << std::setprecision(3) << ";";  // << "\n";
            //cout << I_Right << "\n";
            // cout <<  q_r_dot_Right_fdbk << std::fixed << std::setprecision(3) << ";"; //<< "\n";
            
            //***TORQUE FEEDBACK CALCULATION***//
            // T_mot_Left = Kt*I_Left; //obtain current value and convert to Torque
            // T_mot_Right = Kt*I_Right;

                //add saturation for T_mot_Right since it will be used for our passive torque reference!!
                // if (T_mot_Right > 40){
                //     T_mot_Right = 30;
                // }

            //**debug only:
            // cout <<T_mot_Left << std::fixed << std::setprecision(3) << ";";
            // cout <<T_mot_Right << std::fixed << std::setprecision(3) << ";";
            //cout << T_mot_Left << "\n";
            //cout << T_mot_Right << "\n";

            //note: for this case, let T_mot_Pass = T_mot_Right
            
            // T_hum_act_Left = T_mot_Left - T_mot_Pass; //calculate the active human torque based on static value passive motor torque (no load)  for this case, passive torque using a second motor as reference
            // T_hum_act_Right = T_mot_Right - T_mot_Pass; //line not in use!!
            // T_HRI_Left = T_hum_act_Left; //equate tal HRI 
            // T_HRI_Right = T_hum_act_Right; //line not in use
            
            //debug only:
            // float Tal_left = T_hum_act_Left;
            // float Tal_right = T_hum_act_Right;
            // cout <<Tal_left << std::fixed << std::setprecision(3) << ";";
            // cout <<Tal_right << std::fixed << std::setprecision(3) << ";";

                    /////////////////////////////////////Limiting Current Values "DEAD ZONE"//////////////////////////////////////////////////////////////
            // float I_Pos = 2.85; //[mA] setting upper bound value for current based on experiemental data w/o human active torque
            // float I_Neg = -3.13; //[mA] setting lower bound value for current based on experimental data w/o human active torque

            // //The way it will work is apply a positive or negative corrective torque ramped then delayed for a few seconds then back to zero.
            // if (current_LeftHip > I_Pos){
            //     CurrentDifference = (current_LeftHip - I_Pos)/1000;
            //     LimitTorque = Kt*CurrentDifference;
            //     T_HRI_Left = -LimitTorque;
    
            // }
            // else if (current_LeftHip < I_Neg){
            //     CurrentDifference = (current_LeftHip - I_Neg)/1000; //in this case current_LeftHip would be negative value thus - the I_Neg will give the difference however, because its negative overall, we need negative sign at the end
            //     LimitTorque = Kt*CurrentDifference;
            //     T_HRI_Left = -LimitTorque;
            // }
            // else{
            //     T_HRI_Left = 0;
            // }

            // cout << T_HRI_Left << std::fixed << std::setprecision(3) << ";";
            // cout << current_LeftHip << std::fixed << std::setprecision(3) << ";";


            //***HRI TORQUE AND ENERGY***//
            // T_Left = curr_T_Left_arg + (T_HRI_Left*dt);
            // curr_T_Left_arg = T_Left;
            // E_Left = curr_E_Left_arg + (T_HRI_Left*velocity_LeftHip)*dt;
            // curr_E_Left_arg = E_Left;
            
            // if (E_Left < -10){
            //     E_Left = -7;
            // }
            // else if (E_Left > 10){
            //     E_Left = 7;
            // }
            
            // if (T_Left < -10){
            //     T_Left = -7;
            // }
            // else if (T_Left > 10){
            //     T_Left = 7;
            // }

            
            // cout << T_Left << std::fixed << std::setprecision(3) << ";";
            // cout << E_Left << std::fixed << std::setprecision(3) << ";";


            //cout << "tal_HRI_L: " << T_HRI_Left << " T_HRI_Left: " << T_Left << " E_HRI_Left: " << E_Left << "\n"; 

                //***Left Hip Torque and Energy Saturation***//  !!!right now baesd on testing data but also need the argument to work the negative way
                // if (T_Left > 0.677){
                //     T_Left = 0.677;
                // }
                // if (E_Left > 40){
                //     E_Left = 30;
                // }

            
            //note: right hip not in use!! only second motor used for reference!!
            // T_Right = curr_T_Right_arg + (T_HRI_Right*t);
            // curr_T_Right_arg = T_Right;
            // E_Right = curr_E_Right_arg + (T_HRI_Right*q_r_dot_Right_fdbk)*t;
            // curr_E_Right_arg = E_Right;

            // if (t >= 10 && t <= 14){
            //     T_Left = 0.8;
            //     E_Left = 2;

            // }

            // else 
            //     T_Left = 0;
            //     E_Left = 0;


    
            
            
// ===== Send 1st motor's ID (LEFT HIP)=================================================================== //
            itoa(id[1], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);
            
            q_r_Left = (M_PI/180)*(epsil_Left + rho_Left * (a0_h + a1_h*cos(1*phi_Left) + b1_h*sin(1*phi_Left) + a2_h*cos(2*phi_Left) + 
            b2_h*sin(2*phi_Left) + a3_h*cos(3*phi_Left) + b3_h*sin(3*phi_Left) + a4_h*cos(4*phi_Left) + b4_h*sin(4*phi_Left) + a5_h*cos(5*phi_Left) + 
            b5_h*sin(5*phi_Left) + a6_h*cos(6*phi_Left) + b6_h*sin(6*phi_Left) + a7_h*cos(7*phi_Left)+ b7_h*sin(7*phi_Left) + a8_h*cos(8*phi_Left)+ b8_h*sin(8*phi_Left))); 

            cout << q_r_Left << std::fixed << std::setprecision(3) << ";"; // DEBUG
           

            int rounded_theta = q_r_Left * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);

            // if (T_HRI_Left > 0 || T_HRI_Left < 0){

            //     T_HRI_Left = 0;

            // }

            
// ===== Send 2nd motor's ID (RIGHT HIP)================================================================== //
            itoa(id[2], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            q_r_Right = (M_PI/180)*(epsil_Right + rho_Right * (a0_h + a1_h*cos(1*(phi_Right)) + b1_h*sin(1*(phi_Right)) + a2_h*cos(2*(phi_Right)) + 
            b2_h*sin(2*(phi_Right)) + a3_h*cos(3*(phi_Right)) + b3_h*sin(3*(phi_Right)) + a4_h*cos(4*(phi_Right)) + b4_h*sin(4*(phi_Right)) + a5_h*cos(5*(phi_Right)) + 
            b5_h*sin(5*(phi_Right)) + a6_h*cos(6*(phi_Right)) + b6_h*sin(6*(phi_Right)) + a7_h*cos(7*(phi_Right)) + b7_h*sin(7*(phi_Right)) + a8_h*cos(8*(phi_Right)) + b8_h*sin(8*(phi_Right))-amp_shift_deg)); 
            
            cout << q_r_Right << std::fixed << std::setprecision(3) << ";";// DEBUG
         
            rounded_theta = q_r_Right * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);


// ===== Send 3rd motor's ID (Left Knee)================================================================== //
            itoa(id[3], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n);

            q_r_Left_Knee = (M_PI/180)*(epsil_LeftKnee + rho_LeftKnee * (a0_k + a1_k*cos(1*phi_LeftKnee) + b1_k*sin(1*phi_LeftKnee) + a2_k*cos(2*phi_LeftKnee) + 
            b2_k*sin(2*phi_LeftKnee) + a3_k*cos(3*phi_LeftKnee) + b3_k*sin(3*phi_LeftKnee) + a4_k*cos(4*phi_LeftKnee) + b4_k*sin(4*phi_LeftKnee) + a5_k*cos(5*phi_LeftKnee) + 
            b5_k*sin(5*phi_LeftKnee) + a6_k*cos(6*phi_LeftKnee) + b6_k*sin(6*phi_LeftKnee) + a7_k*cos(7*phi_LeftKnee)+ b7_k*sin(7*phi_LeftKnee) + a8_k*cos(8*phi_LeftKnee)+ b8_k*sin(8*phi_LeftKnee))); 

            // cout << q_r_Left_Knee << std::fixed << std::setprecision(3) << ";";// DEBUG

            rounded_theta = q_r_Left_Knee * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);



// ===== Send 4th motor's ID (Right Knee)================================================================== //

            itoa(id[4], buf, 10);  //use the id at index 1 of id array above
            id_str_n = buf + newline;
            writePort.write(id_str_n); 

            q_r_Right_Knee = (M_PI/180)*(epsil_RightKnee + rho_RightKnee * (a0_k + a1_k*cos(1*(phi_RightKnee)) + b1_k*sin(1*(phi_RightKnee)) + a2_k*cos(2*(phi_RightKnee)) + 
            b2_k*sin(2*(phi_RightKnee)) + a3_k*cos(3*(phi_RightKnee)) + b3_k*sin(3*(phi_RightKnee)) + a4_k*cos(4*(phi_RightKnee)) + b4_k*sin(4*(phi_RightKnee)) + a5_k*cos(5*(phi_RightKnee)) + 
            b5_k*sin(5*(phi_RightKnee)) + a6_k*cos(6*(phi_RightKnee)) + b6_k*sin(6*(phi_RightKnee)) + a7_k*cos(7*(phi_RightKnee)) + b7_k*sin(7*(phi_RightKnee)) + a8_k*cos(8*(phi_RightKnee)) + b8_k*sin(8*(phi_RightKnee))-amp_shift_deg));         

            // cout << q_r_Right_Knee << std::fixed << std::setprecision(3) << ";"; // DEBUG    
      
            rounded_theta = q_r_Right_Knee * 1000;
            itoa(rounded_theta, buf, 10);
            theta_str_n = buf + newline;
            writePort.write(theta_str_n);


            t_prev = corrected_t + milliseconds(dt*step);  //after: t_prev is set to previous element of time array
             
            step++;
        }

        t_now = steady_clock::now();
        running_for = std::chrono::duration<float, std::milli>(t_now - t_pre_loop); //updated

        
    }

    
}
}
return 0;
}