#pragma once

#include "config.h"

enum class ControlState {
    REST,
    ACTIVE,
    TRANSITION
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator();

    /* Feed the voted intent from the classifier. */
    void update(int intent);

    /* Compute target theta (radians) for each joint. */
    void compute(float& m1, float& m2, float& m3);

private:
    float fourier_m1(double t) const;   // Radial / Ulnar
    float fourier_m2(double t) const;   // Flexion / Extension
    float fourier_m3(double t) const;   // Pronation / Supination

    float clamp_delta(float target, float last) const;

    ControlState state_ = ControlState::REST;
    int current_intent_ = cfg::CLASS_REST;
    int prev_intent_    = cfg::CLASS_REST;

    double trajectory_time_ = 0.0;      // monotonic Fourier phase (seconds)
    double dt_              = cfg::INFER_INTERVAL_MS * 0.001;

    float last_m1_ = 0.0f;
    float last_m2_ = 0.0f;
    float last_m3_ = 0.0f;

    int   blend_tick_  = 0;              // transition counter
    float blend_alpha_ = 1.0f;
    float old_m1_ = 0.0f, old_m2_ = 0.0f, old_m3_ = 0.0f;

    // Fourier coefficients — ported from serial_write_threejoints_wrist
    static constexpr float a0=40.69f, a1=23.22f, b1_=-8.65f, a2=-4.487f, b2_=3.338f;
    static constexpr float a3=0.3995f, b3_=1.389f, a4=0.7047f, b4_=0.7989f;
    static constexpr float a5=1.078f, b5_=0.342f, a6=-0.2732f, b6_=0.06696f;
    static constexpr float w_=0.65f;

    static constexpr float ka0=25.7f, ka1=-3.83f, kb1=-19.28f, ka2=-8.54f, kb2=17.93f;
    static constexpr float ka3=1.91f, kb3=3.77f, ka4=1.09f, kb4=1.50f;
    static constexpr float ka5=2.05f, kb5=0.58f, ka6=-0.31f, kb6=-0.90f;

    static constexpr float amp_shift = 23.0f;  // degrees
    static constexpr float amp_m1 = 1.25f;
    static constexpr float amp_m2 = 2.0f;
    static constexpr float amp_m3 = 7.0f;
    static constexpr float phase_shift_deg = 180.0f;
};
