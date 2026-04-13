/************************************************************************************
 //after double motors!
 
 This program is used to control the motors in the 'Follow a Trajectory'
 mode where the trajectory can be a simple cosine function or a complex
 trajectory like that of a human hip while walking.

 In this mode, you can record the motor's position as it follows the
 reference trajectory and plot it to verify.

 By: Sai Hein Si Thu
 Updated: 2023-MAY-06 1850
************************************************************************************/

/**********************          WHEN UPLOADING CODE          **********************/
// If you're uploading this code to ESP32 from VS Code, you must hold the EN
// button. Orient your ESP so that the microUSB port is on the top. The EN
// button will then be the button on the left of the mciroUSB port. Hold the
// button while uploading the code. You can let go of the button when you see
// "Writing (xx %)...." in the terminal.


#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <stdint.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <string>
#include <HardwareSerial.h>

using namespace std;


/*******  Global Variables  *******/
//CAN_frame_t tx_frame;
CAN_device_t CAN_cfg;           // for configuring our CAN comm
const int rx_queue_size = 10;   // num of CAN msgs that can be in queue
// Following variables are used in the loop() func below
//long t_pre_loop;  // can be declared as static var later: t_pre_loop, now, last_time
//long now;
//long last_time;
//int run_time = 3*PI * pow(10, 6);      // run for x seconds (s)
//float dt = 20 * pow(10, 3);           // dt required between each motor command, in ms
//const int SIZE = (run_time / dt) + 1;  // one extra just in case


/*******  Constants  *******/
// Same for these motors: AK10-9, 60-6, 70-10, 80-6, 80-9, 80-80/64
const float P_MIN = -12.5;
const float P_MAX = 12.5;
const float Kp_MIN = 0;
const float Kp_MAX = 500;
const float Kd_MIN = 0;
const float Kd_MAX = 5;
const float Test_Pos= 0.0;

//For AK80-64 motor:
const float V_MIN = -8;
const float V_MAX = 8;
const float T_MIN = -144;
const float T_MAX = 144;

// For AK70-10 motors:
//const float V_MIN = -50;
//const float V_MAX = 50;
//const float T_MIN = -25;
//const float T_MAX = 25;



/*******************************************************************
 Definition of float_to_uint function

 This function convertes a float to an unsigned int given range and 
 number of bits of the uint number. Conversion to [uint] needs to be
 done before sending the data to the motor.

 It is basically an interpolation calculator where value x from one
 range: [x_min, x_max] is mapped to another range: [0, 2^bits].
*******************************************************************/
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  
  // '1 << bits' is simply 2^bits, it's multiplying 1 by 2^bits
  return (int) ((x- x_min)*((float)((1<<bits)/span)));
}



/*******************************************************************
 Definition of pack_cmd function

 This function packs user's input command into a CAN message, which
 can be sent to the motor using another function.
*******************************************************************/
void pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq) {
  // Limits data to be within bounds
  float p_flt = fminf(fmaxf(P_MIN, p), P_MAX);
  float v_flt = fminf(fmaxf(V_MIN, v), V_MAX);
  float kp_flt = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  float kd_flt = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  float t_flt = fminf(fmaxf(T_MIN, torq), T_MAX);
  
  // Converts floats to unsigned ints [uint]
  int p_uint = float_to_uint(p_flt, P_MIN, P_MAX, 16);
  int v_uint = float_to_uint(v_flt, V_MIN, V_MAX, 12);
  int kp_uint = float_to_uint(kp_flt, Kp_MIN, Kp_MAX, 12);
  int kd_uint = float_to_uint(kd_flt, Kd_MIN, Kd_MAX, 12);
  int t_uint = float_to_uint(t_flt, T_MIN, T_MAX, 12);

  // Pack [uint] values into CAN message in accordance with the motor's
  // "receive data definition" from the motor manual (p.48-49)
  // 0xF is 1111, 0xFF is 11111111, each F appends 1111; F or f doesnt matter
  msg->data.u8[0] = p_uint>>8;                       // Position 8 higher
  msg->data.u8[1] = p_uint&0xFF;                     // Position 8 lower
  msg->data.u8[2] = v_uint>>4;                       // Speed 8 higher
  msg->data.u8[3] = ((v_uint&0xF)<<4)|(kp_uint>>8);  // Speed 4 bit lower Kp 4 bit higher
  msg->data.u8[4] = kp_uint&0xFF;                    // Kp 8 bit lower
  msg->data.u8[5] = kd_uint>>4;                      // Kd 8 bit higher
  msg->data.u8[6] = ((kd_uint&0xF)<<4)|(t_uint>>8);  // Kd 4 bit lower torque 4 bit higher
  msg->data.u8[7] = t_uint&0xff;                     // torque 8 bit lower
}



/*******************************************************************
 Definition of uint_to_float function

 This function converts an [uint] value sent from the motor to a
 float value given the range and number of bits of the uint number.
 It does interpolation like the float_to_uint() func as well.
*******************************************************************/
float uint_to_float(int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;

  /* This function is performing as expected. uint to float conversion
  has been tested. Data obtained from test is shown below:
  1. p_int = 38086 -> 2.03   (expected value = 2.029)
  2. v_int = 2818  -> 3.01   (expected value = -0.002)
  3. t_int = 2040  -> -0.53  (expected value = -0.527)
  */
}



/*******************************************************************
 Definition of unpack_reply function
 
 This function unpacks the reply from the motor. 
 
 Parameters:
 CAN_frame_t msg:
   This is the motor's return msg. It is not passed as reference since
   we do not need to modify the msg itself, we only need a copy of
   its content.

 state[]:
   The motor's position, speed, and torque will be stored in an arrary
   named 'state'.
*******************************************************************/
void unpack_reply(CAN_frame_t msg, float state[]){
  // Deconstructing motor's return message, which are [uint] values
  int id = msg.data.u8[0];                                // Motor ID
  int p_uint = (msg.data.u8[1]<<8)|msg.data.u8[2];        // Motor position data
  int v_uint = (msg.data.u8[3]<<4)|(msg.data.u8[4]>>4);   // Motor speed data
  int t_uint = ((msg.data.u8[4]&0xF)<<8)|msg.data.u8[5];  // Motor torque data

  /* As stated in the manual (p.49), the motor return code is 6 bytes. 
   DATA[6] and DATA[7], which are motor temp and error code, always
   return as 0. */
  //int temp_int = msg.data.u8[6];     // Motor temperature
  //int errcode_int = msg.data.u8[7];  // Error code

  // Convert to float and store in array named state[]
  state[0] = id;  //newly added, might cause problems
  state[1] = uint_to_float(p_uint, P_MIN, P_MAX, 16);
  state[2] = uint_to_float(v_uint, V_MIN, V_MAX, 12);
  state[3] = uint_to_float(t_uint, T_MIN, T_MAX, 12);
}



/***************************************************************
 Definition of to_motors function

 Send commands to the motor specified by ID. The ID of a motor can
 be set using CubeMars' software.

 Parameters and Variables:
 int id:
    This is the ID of the motor that the CAN message will go to. It
    can be set from 0 to 0x7FF (2047); the HIGHER the value,
    the LOWER the priority of the motor. It's only till 2047
    because AK80-64 only supports MIT mode, which uses the Standard
    Frame CAN msg, which has an 11-bit identifier (ID). 2047 is the
    largest 11-bit number. Extended Frame CAN msg has a 29-bit
    identifier (ID) so it can go more than 2047.
 
 FIR.B.DLC:
    This is the size of the data frame in a CAN message (in bytes).
    Motor manual outlined "receives data definition" which is how
    the motor recives the CAN msg. So, that is how we have to format
    our CAN message. DLC = 8 bytes in this case and 6 bytes for 
    "send data definition" which is how the motor sends back a reply.
***************************************************************/
void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg){
  // Define properties of the CAN msg that will be sent
  msg->FIR.B.FF = CAN_frame_std;  // Standard or extended CAN frame
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;  // MIT mode command is 8 bytes, see "receives data definition" (p.48)

  // Insert command into the CAN msg
  pack_cmd(msg, pos, vel, kp, kd, torq);
  
  // The function CANWriteFrame() sends the CAN mesg to ESP32's CAN controller
  ESP32Can.CANWriteFrame(msg);
}



/*******************************************************************
 Definition of enter_control_mode

 Puts the motor specified by ID into control mode. 
*******************************************************************/
void enter_control_mode(int id, CAN_frame_t *msg) {
  // Sending the following CAN msg puts the motor into control mode.
  // In other words, this is the first command you send to make the 
  // motor start listening for other commands. (Source: p.48)
  msg->FIR.B.FF = CAN_frame_std;
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;
  msg->data.u8[0] = 0xFF;
  msg->data.u8[1] = 0xFF;
  msg->data.u8[2] = 0xFF;
  msg->data.u8[3] = 0xFF;
  msg->data.u8[4] = 0xFF;
  msg->data.u8[5] = 0xFF;
  msg->data.u8[6] = 0xFF;
  msg->data.u8[7] = 0xFC;
  ESP32Can.CANWriteFrame(msg);
}



int num;
CAN_frame_t tx_frame;  // tx_frame is the variable tht represents the CAN msg to be sent to motors
//int id = 2;  // ID of the motor you want to control
//int id2 = 4;  //id values commented out
HardwareSerial mySerial2(2);  // DEBUG serial port 2
//const int runtime = 20;
//const int dt = 100 * pow(10, -3);
const int SIZE = 2000;  //change this according to runtime and dt in .py script
//float command_theta[SIZE];
//float encoder_theta[SIZE];
int step = 0;
int num_of_motors;
int motor_id[3];  //change 10 to however many motors you want to connect to ESP


/*******************************************************************
 Set up function for the ESP32
*******************************************************************/
void setup() {
  Serial.begin(115200);                  // VS code's default is 9600
  
  // Serial for debugging purposes
  mySerial2.begin(115200, SERIAL_8N1, 17, 18); // FTDI RX to ESP TX (pin 18)

  CAN_cfg.speed = CAN_SPEED_1000KBPS;  // matches manual-specified 1 Mbps CAN bus speed
  CAN_cfg.tx_pin_id = GPIO_NUM_5;      // TX-RX pins might vary based on ESP32 model
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));  // data receive queue
  
  ESP32Can.CANInit();  // Initialize CAN Module
  
  // Put the motors into control mode, shown below are motors w/ ID = 1 and ID = 2
  //enter_control_mode(id, &tx_frame);
  //enter_control_mode(id2, &tx_frame);

  // Populate an array with all connected motors' IDs so that they can all be
  // put into listening mode later
  //while (Serial.available()==0) {}  //gets info on how many motor we're controlling
  //String num_motors_str = Serial.readStringUntil('\n');
  //num_of_motors = atoi(num_motors_str.c_str());

  //for (int i = 0; i < num_of_motors; i++){
  //  while (Serial.available()==0) {}
  //  String id_str = Serial.readStringUntil('\n');
  //  motor_id[i] = atoi(id_str.c_str());
  //  enter_control_mode(motor_id[i], &tx_frame);
  //  //mySerial2.println(motor_id[i]);  //remove later
  //}

  enter_control_mode(1, &tx_frame);
  mySerial2.println("1 in control");
  delay(1000);

  enter_control_mode(2, &tx_frame);
  mySerial2.println("2 in control");
  delay(1000);

  enter_control_mode(3, &tx_frame);
  mySerial2.println("3 in control");
  delay(1000);

  enter_control_mode(4, &tx_frame);
  mySerial2.println("4 in control");
  delay(1000);

  enter_control_mode(5, &tx_frame);
  mySerial2.println("5 in control");
  delay(1000);

  enter_control_mode(6, &tx_frame);
  mySerial2.println("6 in control");
  delay(1000);

}



/*******************************************************************
 Loop function for the ESP32
*******************************************************************/
void loop() {
  // Need dynamic memory alloc since 'SIZE' is not a fixed constant;
  // it can change based on dt and run_time, both of which are arbitrary
  //float* command_theta = new float[SIZE];
  //float* encoder_theta = new float[SIZE];

  // DEBUG for loop
  //for (int i = 0; i < num_of_motors; i++){
  //  mySerial2.println(motor_id[i]);
  //}
  //mySerial2.println("test");

  CAN_frame_t motor_returnMsg1;
  CAN_frame_t motor_returnMsg2;  //prob only need one returnMsg

  // For walking at freq 18*w, Kp=420 and Kd=4.5 were the best values.
  int kp = 50;  //set back to 495, 420
  int kd = 2;  //set back to 4.5, 4.5
  int step = 0;  // keeps track of the number of commands sent
  float motor_state[3];  // motor's position, velocity, and torque
  

  // Get motor ID
  while (Serial.available() == 0){}
  String id_str = Serial.readStringUntil('\n');
  int id = atoi(id_str.c_str());
  //mySerial2.println(id1);
  //mySerial2.println("printed");

  // Get theta_desired
  while (Serial.available()==0) {}  //wait for theta_desired
  String theta_str = Serial.readStringUntil('\n');
  //String theta_str = Serial.readString();
  int theta_int = atoi(theta_str.c_str());
  float theta_d = theta_int / 1000.000;  //int divided by int will give an int, so has to be 1000.000

  // id, pos, vel, kp, kd, torq, CAN msg
  to_motor(id, theta_d, 0, kp, kd, 0, &tx_frame);  //theta_d is in radians
  

  // The motor will give feedback for each command it receives
  if(xQueueReceive(CAN_cfg.rx_queue, &motor_returnMsg1, 1*portTICK_PERIOD_MS)==pdTRUE){
    unpack_reply(motor_returnMsg1, motor_state);  //modified 'unpack_reply' such that state[0] is now the ID
    // encoder_theta[step] = motor_state[0];  //only want position from motor_state; position is now motor_state[1] actually
  }

  // DEBUG printing command theta values
  //mySerial2.printf("theta_d: %.3f,\r\n", theta_d);
  //mySerial2.print("ID: ");
  //mySerial2.print(motor_state[0]);
  //mySerial2.printf(", enc: %.3f\r\n\n", motor_state[0], motor_state[1]);

  // Reminder: mySerial2.printxxx() sends data to the PC's COM port. 
  // Sending as string with +/- check
  //if(motor_state[1] >= 0) mySerial2.printf("%.4f", motor_state[1]);
  //else mySerial2.printf("%.3f", motor_state[1]);

  // Simple printing
  mySerial2.printf("%.4f;", motor_state[1]);

  // Sending data as binary
  //int data_int = motor_state[1] * 1000;
  //float data = data_int/1000.0;
  //byte *dataPointer = (byte *)&data;
  //int dataSize = sizeof(data);
  //mySerial2.write(dataPointer, dataSize);

  //float enc = motor_state[1];
  ////float data = data_int/1000.0;
  //byte *dataPointer = (byte *)&enc;
  //int dataSize = sizeof(enc);
  //mySerial2.write(dataPointer, dataSize);

}

/*
  

  // DEBUG printing encoder theta values
  //mySerial2.printf("%.3f,\r\n", encoder_theta[step]);
  if(motor_state[0] >= 0) mySerial2.printf("%.4f", motor_state[0]);
  else mySerial2.printf("%.3f", motor_state[0]);  // CHANGE BACK to "%.3f\r\n" if needed

  to_motor(id2, theta_d, 0, kp, kd, 0, &tx_frame);

  //command_theta[step] = theta_d;

  step++;
    

  //Serial.print("step = ");
  //Serial.println(step); 


  //Serial.println("\n\n================ BELOW IS COMMAND THETA ========================");
  //for (int i=0; i <= step; i++) {
  //  //Serial.printf("%.3f,", command_theta[i]);  // prints w 3 dec places
  //  // do not add a 'space' after ',' because that adds random '\n' making it hard to use data in matlab
  //  // if you do println, it truncates the float to 2 dec places
  //  // if you do %d, it prints weird numbers
  //}

  //Serial.println("\n\n================ BELOW IS ENCODER THETA ========================");
  //for (int i=0; i <= step; i++) {
    //Serial.printf("%.3f,", encoder_theta[i]);
  //}



  // Free up the memory
  //delete[] command_theta;
  //delete[] encoder_theta;
}  //this is the end of the program (2023-04-25)
*/