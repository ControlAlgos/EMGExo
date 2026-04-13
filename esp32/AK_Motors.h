/*******************************************************************************
 Header file for motor-related helper functions.

 By: Sai Hein Si Thu
 Updated: 2023-JUN-14
*******************************************************************************/
#pragma once

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
void pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void unpack_reply(CAN_frame_t msg, float state[]);
void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg);
void enter_control_mode(int id, CAN_frame_t *msg);
void set_origin(int id, CAN_frame_t *msg);
