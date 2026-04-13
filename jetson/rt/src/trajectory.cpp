#include "trajectory.h"
#include <cmath>
#include <algorithm>

static constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);
static constexpr float DECAY = 0.995f;

// ─── Gesture → Motor mapping ────────────────────────────────────────────
//  WRIST_FLEX / WRIST_EXT   → M2 only  (Flexion/Extension)
//  PRONATION  / SUPINATION  → M3 only  (Pronation/Supination)
//  MASS_FLEX  / MASS_EXT    → M1 + M2  (Radial/Ulnar + Flex/Ext combined)
//  REST                     → all decay to zero

struct MotorMask { bool m1, m2, m3; };

static MotorMask gesture_to_motors(int intent) {
    switch (intent) {
        case cfg::CLASS_WRIST_FLEX:
        case cfg::CLASS_WRIST_EXT:  return {false, true,  false};
        case cfg::CLASS_PRONATION:
        case cfg::CLASS_SUPINATION: return {false, false, true};
        case cfg::CLASS_MASS_FLEX:
        case cfg::CLASS_MASS_EXT:   return {true,  true,  false};
        default:                    return {false, false, false};
    }
}

TrajectoryGenerator::TrajectoryGenerator() {
    last_m1_ = 0.0f;
    last_m2_ = 0.0f;
    last_m3_ = 0.0f;
}

// ─── Fourier evaluators (ported from serial_write_threejoints_wrist) ────

float TrajectoryGenerator::fourier_m1(double t) const {
    float tw = static_cast<float>(t) * w_;
    return cfg::AMPLITUDE_SCALE * DEG2RAD * amp_m1 * (
        a0 + a1*cosf(tw)     + b1_*sinf(tw)
           + a2*cosf(2*tw)   + b2_*sinf(2*tw)
           + a3*cosf(3*tw)   + b3_*sinf(3*tw)
           + a4*cosf(4*tw)   + b4_*sinf(4*tw)
           + a5*cosf(5*tw)   + b5_*sinf(5*tw)
           + a6*cosf(6*tw)   + b6_*sinf(6*tw)
           - amp_shift);
}

float TrajectoryGenerator::fourier_m2(double t) const {
    float traj_period = 2.0f * static_cast<float>(M_PI) / w_;
    float b_shift = (phase_shift_deg / 360.0f) * traj_period;
    float tw = static_cast<float>(t - b_shift) * w_;
    return cfg::AMPLITUDE_SCALE * -DEG2RAD * amp_m2 * (
        a0 + a1*cosf(tw)     + b1_*sinf(tw)
           + a2*cosf(2*tw)   + b2_*sinf(2*tw)
           + a3*cosf(3*tw)   + b3_*sinf(3*tw)
           + a4*cosf(4*tw)   + b4_*sinf(4*tw)
           + a5*cosf(5*tw)   + b5_*sinf(5*tw)
           + a6*cosf(6*tw)   + b6_*sinf(6*tw)
           - amp_shift);
}

float TrajectoryGenerator::fourier_m3(double t) const {
    float tw = static_cast<float>(t) * w_;
    return cfg::AMPLITUDE_SCALE * -DEG2RAD * amp_m3 * (
        ka0 + ka1*cosf(tw)   + kb1*sinf(tw)
            + ka2*cosf(2*tw) + kb2*sinf(2*tw)
            + ka3*cosf(3*tw) + kb3*sinf(3*tw)
            + ka4*cosf(4*tw) + kb4*sinf(4*tw)
            + ka5*cosf(5*tw) + kb5*sinf(5*tw)
            + ka6*cosf(6*tw) + kb6*sinf(6*tw)
            - amp_shift);
}

float TrajectoryGenerator::clamp_delta(float target, float last) const {
    float delta = target - last;
    if (delta >  cfg::MAX_DELTA_RAD) delta =  cfg::MAX_DELTA_RAD;
    if (delta < -cfg::MAX_DELTA_RAD) delta = -cfg::MAX_DELTA_RAD;
    return last + delta;
}

// ─── State machine ──────────────────────────────────────────────────────

void TrajectoryGenerator::update(int intent) {
    prev_intent_ = current_intent_;
    current_intent_ = intent;

    if (intent == cfg::CLASS_REST) {
        state_ = ControlState::REST;
        return;
    }

    if (state_ == ControlState::REST) {
        state_ = ControlState::TRANSITION;
        blend_tick_ = 0;
        blend_alpha_ = 0.0f;
        old_m1_ = last_m1_;
        old_m2_ = last_m2_;
        old_m3_ = last_m3_;
        return;
    }

    if (state_ == ControlState::TRANSITION) {
        blend_tick_++;
        blend_alpha_ += cfg::BLEND_STEP;
        if (blend_alpha_ >= 1.0f) {
            blend_alpha_ = 1.0f;
            state_ = ControlState::ACTIVE;
        }
    }
}

void TrajectoryGenerator::compute(float& m1, float& m2, float& m3) {
    if (state_ == ControlState::REST) {
        last_m1_ *= DECAY;
        last_m2_ *= DECAY;
        last_m3_ *= DECAY;
        m1 = last_m1_; m2 = last_m2_; m3 = last_m3_;
        return;
    }

    trajectory_time_ += dt_;

    MotorMask mask = gesture_to_motors(current_intent_);

    // Active motors follow their Fourier trajectory; inactive ones decay to zero
    float target_m1 = mask.m1 ? fourier_m1(trajectory_time_) : last_m1_ * DECAY;
    float target_m2 = mask.m2 ? fourier_m2(trajectory_time_) : last_m2_ * DECAY;
    float target_m3 = mask.m3 ? fourier_m3(trajectory_time_) : last_m3_ * DECAY;

    if (state_ == ControlState::TRANSITION) {
        float a = blend_alpha_;
        target_m1 = old_m1_ * (1.0f - a) + target_m1 * a;
        target_m2 = old_m2_ * (1.0f - a) + target_m2 * a;
        target_m3 = old_m3_ * (1.0f - a) + target_m3 * a;
    }

    m1 = clamp_delta(target_m1, last_m1_);
    m2 = clamp_delta(target_m2, last_m2_);
    m3 = clamp_delta(target_m3, last_m3_);

    last_m1_ = m1;
    last_m2_ = m2;
    last_m3_ = m3;
}
