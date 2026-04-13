#pragma once

#include <cstdint>
#include <cstring>

/*
 * 14-byte packed binary command sent to ESP32.
 * Layout:
 *   [0]      start        0xA5
 *   [1]      joint_id     1..3
 *   [2..5]   target_theta float, little-endian
 *   [6..9]   kp           float, little-endian
 *   [10..13] kd           float, little-endian
 *   [14]     checksum     XOR of bytes [1..13] (excludes start byte)
 *
 * Total: 15 bytes per joint, 45 bytes for 3 joints.
 */

#pragma pack(push, 1)
struct MotorCommand {
    uint8_t start;
    uint8_t joint_id;
    float   target_theta;
    float   kp;
    float   kd;
    uint8_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(MotorCommand) == 15, "MotorCommand must be 15 bytes");

inline MotorCommand make_command(uint8_t id, float theta, float kp, float kd) {
    MotorCommand cmd;
    cmd.start        = 0xA5;
    cmd.joint_id     = id;
    cmd.target_theta = theta;
    cmd.kp           = kp;
    cmd.kd           = kd;

    uint8_t xor_val = 0;
    const auto* p = reinterpret_cast<const uint8_t*>(&cmd);
    // XOR bytes [1..13]: joint_id, target_theta, kp, kd (skip start byte)
    for (size_t i = 1; i < sizeof(MotorCommand) - 1; ++i)
        xor_val ^= p[i];
    cmd.checksum = xor_val;

    return cmd;
}
