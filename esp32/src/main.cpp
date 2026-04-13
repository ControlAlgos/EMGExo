/*
 * ESP32 Motor Controller — Simple Text Protocol @ 921600 baud
 * Matches the original working firmware, just faster baud + faster setup.
 */

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <stdint.h>
#include <math.h>
#include <HardwareSerial.h>

CAN_device_t CAN_cfg;
const int rx_queue_size = 10;

const float P_MIN = -12.5, P_MAX = 12.5;
const float Kp_MIN = 0, Kp_MAX = 500;
const float Kd_MIN = 0, Kd_MAX = 5;
const float V_MIN = -8, V_MAX = 8;
const float T_MIN = -144, T_MAX = 144;

int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if (x < x_min) x = x_min;
  else if (x > x_max) x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) / span)));
}

void pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq) {
  int p_uint  = float_to_uint(fminf(fmaxf(P_MIN, p), P_MAX), P_MIN, P_MAX, 16);
  int v_uint  = float_to_uint(fminf(fmaxf(V_MIN, v), V_MAX), V_MIN, V_MAX, 12);
  int kp_uint = float_to_uint(fminf(fmaxf(Kp_MIN, kp), Kp_MAX), Kp_MIN, Kp_MAX, 12);
  int kd_uint = float_to_uint(fminf(fmaxf(Kd_MIN, kd), Kd_MAX), Kd_MIN, Kd_MAX, 12);
  int t_uint  = float_to_uint(fminf(fmaxf(T_MIN, torq), T_MAX), T_MIN, T_MAX, 12);
  msg->data.u8[0] = p_uint >> 8;
  msg->data.u8[1] = p_uint & 0xFF;
  msg->data.u8[2] = v_uint >> 4;
  msg->data.u8[3] = ((v_uint & 0xF) << 4) | (kp_uint >> 8);
  msg->data.u8[4] = kp_uint & 0xFF;
  msg->data.u8[5] = kd_uint >> 4;
  msg->data.u8[6] = ((kd_uint & 0xF) << 4) | (t_uint >> 8);
  msg->data.u8[7] = t_uint & 0xff;
}

void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg) {
  msg->FIR.B.FF = CAN_frame_std;
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;
  pack_cmd(msg, pos, vel, kp, kd, torq);
  ESP32Can.CANWriteFrame(msg);
}

void enter_control_mode(int id, CAN_frame_t *msg) {
  msg->FIR.B.FF = CAN_frame_std;
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;
  msg->data.u8[0] = 0xFF; msg->data.u8[1] = 0xFF;
  msg->data.u8[2] = 0xFF; msg->data.u8[3] = 0xFF;
  msg->data.u8[4] = 0xFF; msg->data.u8[5] = 0xFF;
  msg->data.u8[6] = 0xFF; msg->data.u8[7] = 0xFC;
  ESP32Can.CANWriteFrame(msg);
}

CAN_frame_t tx_frame;

void setup() {
  Serial.begin(921600);

  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  for (int id = 1; id <= 6; id++) {
    enter_control_mode(id, &tx_frame);
    delay(500);
  }

  // Drain any serial data that arrived during setup
  while (Serial.available() > 0) Serial.read();

  Serial.println("READY");
}

void loop() {
  CAN_frame_t motor_returnMsg;
  float motor_state[4];

  while (Serial.available() == 0) {}
  String id_str = Serial.readStringUntil('\n');
  int id = atoi(id_str.c_str());

  while (Serial.available() == 0) {}
  String theta_str = Serial.readStringUntil('\n');
  int theta_int = atoi(theta_str.c_str());
  float theta_d = theta_int / 1000.000;

  to_motor(id, theta_d, 0, 50, 2, 0, &tx_frame);

  if (xQueueReceive(CAN_cfg.rx_queue, &motor_returnMsg, 1 * portTICK_PERIOD_MS) == pdTRUE) {
    float state[4];
    (void)state;
  }
}
