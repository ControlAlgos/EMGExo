/*******************************************************************************
 Source file for motor-related helper functions. These functions are based off 
 of the information provided in the motor manual.

 By: Sai Hein Si Thu
 Updated: 2023-JUN-14
*******************************************************************************/
#include "AK_Motors.h"


/**********  Constants  **********/
// Same for these motors: AK10-9, 60-6, 70-10, 80-6, 80-9, 80-80/64
const float P_MIN = -12.5;  //see motor manual (p. 49-50) for these constants
const float P_MAX = 12.5;
const float Kp_MIN = 0;
const float Kp_MAX = 500;
const float Kd_MIN = 0;
const float Kd_MAX = 5;
const float Test_Pos = 0.0;

//For AK80-64 motor:
//const float V_MIN = -8;
//const float V_MAX = 8;
//const float T_MIN = -144;
//const float T_MAX = 144;

// For AK70-10 motors:
const float V_MIN = -50;
const float V_MAX = 50;
const float T_MIN = -25;
const float T_MAX = 25;


/*******************************************************************************
 Definition of float_to_uint function

 This function converts a float to an unsigned int given range and 
 number of bits of the uint number. Conversion to [uint] needs to be
 done before sending the data to the motor.

 It is basically an interpolation calculator where value x from one
 range: [x_min, x_max] is mapped to another range: [0, 2^bits].
*******************************************************************************/
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  
  // '1 << bits' is simply 2^bits, it's multiplying 1 by 2^bits
  return (int) ((x- x_min)*((float)((1<<bits)/span)));
}



/*******************************************************************************
 Definition of pack_cmd function

 This function packs user's input command into a CAN message, which
 can be sent to the motor using another function.
*******************************************************************************/
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

  /*
   Pack [uint] values into CAN message in accordance with the motor's
   "receive data definition" from the motor manual (p.48-49)
   0xF is 1111, 0xFF is 11111111, each F appends 1111; F or f doesnt matter.
  */
  msg->data.u8[0] = p_uint>>8;                       // Position 8 higher
  msg->data.u8[1] = p_uint&0xFF;                     // Position 8 lower
  msg->data.u8[2] = v_uint>>4;                       // Speed 8 higher
  msg->data.u8[3] = ((v_uint&0xF)<<4)|(kp_uint>>8);  // Speed 4 bit lower Kp 4 bit higher
  msg->data.u8[4] = kp_uint&0xFF;                    // Kp 8 bit lower
  msg->data.u8[5] = kd_uint>>4;                      // Kd 8 bit higher
  msg->data.u8[6] = ((kd_uint&0xF)<<4)|(t_uint>>8);  // Kd 4 bit lower torque 4 bit higher
  msg->data.u8[7] = t_uint&0xff;                     // torque 8 bit lower
}



/*******************************************************************************
 Definition of uint_to_float function

 This function converts an [uint] value sent from the motor to a
 float value given the range and number of bits of the uint number.
 It does interpolation like the float_to_uint() func as well.
*******************************************************************************/
float uint_to_float(int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;

  /*
   This function is performing as expected. uint to float conversion
   has been tested. Data obtained from test is shown below:
   1. p_int = 38086 -> 2.03   (expected value = 2.029)
   2. v_int = 2818  -> 3.01   (expected value = -0.002)
   3. t_int = 2040  -> -0.53  (expected value = -0.527)
  */
}



/*******************************************************************************
 Definition of unpack_reply function
 
 This function unpacks the feedback (reply) message sent by the motor. 
 
 Parameters:
 CAN_frame_t msg:
   This is the motor's return msg. It is not passed as reference since
   we do not need to modify the msg itself, we only need a copy of
   its content.

 state[]:
   The motor's id, position, speed, and torque will be stored in an arrary
   named 'state'.
*******************************************************************************/
void unpack_reply(CAN_frame_t msg, float state[]){
  // Deconstructing motor's return message, which are [uint] values
  int id = msg.data.u8[0];                                // Motor ID
  int p_uint = (msg.data.u8[1]<<8)|msg.data.u8[2];        // Motor position data
  int v_uint = (msg.data.u8[3]<<4)|(msg.data.u8[4]>>4);   // Motor speed data
  int t_uint = ((msg.data.u8[4]&0xF)<<8)|msg.data.u8[5];  // Motor torque data

  /*
   As stated in the manual (p.49), the motor return code is 6 bytes. 
   DATA[6] and DATA[7], which are motor temp and error code, always
   return as 0, so commented out. Perhaps there is a better way to retrive
   these values.
  */
  //int temp_int = msg.data.u8[6];     // Motor temperature
  //int errcode_int = msg.data.u8[7];  // Error code

  // Convert to float and store in array named state[]
  state[0] = id;
  state[1] = uint_to_float(p_uint, P_MIN, P_MAX, 16);
  state[2] = uint_to_float(v_uint, V_MIN, V_MAX, 12);
  state[3] = uint_to_float(t_uint, T_MIN, T_MAX, 12);
}



/*******************************************************************************
 Definition of to_motors function

 Send commands to the motor specified by 'id'. The ID of a motor can
 be set using CubeMars' software called 'CubeMarstool_V1.32.exe' (find it in the 
 shared drive since the official website's copy is corrupted as of 2023-06-13). 
 You need to connect PC to the motor using an R-Link in order to interface with 
 the motor using CubeMarstool.

 Parameters and Variables:
 int id:
    This is the ID of the motor that the CAN message will be sent to. It
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
*******************************************************************************/
void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg){
  // Define properties of the CAN msg that will be sent
  msg->FIR.B.FF = CAN_frame_std;  // Standard or extended CAN frame
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;  // MIT mode command is 8 bytes, \
                          see "receives data definition" (p.48)

  // Insert command into the CAN msg
  pack_cmd(msg, pos, vel, kp, kd, torq);
  
  // The function CANWriteFrame() sends the CAN msg to the motor by first sending\
     it to ESP32's CAN controller, which then sends to the CAN transceiver.
  ESP32Can.CANWriteFrame(msg);
}



/*******************************************************************************
 Definition of enter_control_mode

 Puts the motor specified by ID into control mode (aka listening mode). 
*******************************************************************************/
void enter_control_mode(int id, CAN_frame_t *msg) {
  /*
   Sending the following CAN msg puts the motor into control mode.
   In other words, this is the first command you send to make the 
   motor start listening for other commands. (See motor manual p.48)
  */
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



/*******************************************************************************
 Definition of set_origin

 Sets the current position of the motor to be the origin
 (i.e. 0 rad position). 
*******************************************************************************/
void set_origin(int id, CAN_frame_t *msg) {
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
  msg->data.u8[7] = 0xFE;
  ESP32Can.CANWriteFrame(msg);
}
