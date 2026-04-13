#pragma once

#include "config.h"
#include <cstdint>

class Hud {
public:
    Hud();
    ~Hud();

    Hud(const Hud&) = delete;
    Hud& operator=(const Hud&) = delete;

    void render(int intent, float confidence,
                float m1, float m2, float m3,
                double total_us, double infer_us,
                const char* state_str,
                int streamer_gesture = -1,
                const float rms[3] = nullptr,
                double uart_us = 0.0,
                const char* mode_label = nullptr,
                bool autoplay_on = false,
                int autoplay_gesture = 0,
                float autoplay_progress = 0.0f,
                float autoplay_elapsed_s = 0.0f,
                float autoplay_total_s = 0.0f);

    /* Push raw EMG window (channel-major, pre-normalization) for oscilloscope view. */
    void push_raw_emg(const float* channel_major_buf);

    /* Push per-channel RMS for rolling envelope. */
    void push_emg_rms(const float rms[3]);

    /* Track CNN accuracy vs ground truth from autoplay schedule. */
    void record_classification(int ground_truth, int predicted);

    void record_latency(double loop_us);

    double avg_latency_ms() const { return avg_loop_ms_; }
    double max_latency_ms() const { return max_loop_ms_; }
    double min_latency_ms() const { return min_loop_ms_; }
    int    total_ticks()    const { return tick_count_; }
    double session_seconds() const;

    static const char* intent_name(int id);

private:
    bool   first_ = true;

    // Latency tracking
    double avg_loop_ms_ = 0.0;
    double max_loop_ms_ = 0.0;
    double min_loop_ms_ = 999.0;
    int    tick_count_   = 0;
    double sum_loop_ms_  = 0.0;

    // Raw EMG waveform (latest 205-sample window per channel)
    static constexpr int WAVE_DISP = 50;  // display columns for waveform
    float  raw_window_[cfg::NUM_CHANNELS][cfg::WINDOW_SIZE] = {};
    bool   raw_valid_ = false;

    // Rolling RMS envelope history
    static constexpr int SPARK_LEN = 50;
    static constexpr int SUBSAMPLE_RATE = 4;   // push every 4th tick (20ms)
    float  emg_hist_[3][SPARK_LEN] = {};
    int    hist_write_ = 0;
    int    hist_count_ = 0;
    int    subsample_  = 0;

    // Classification accuracy
    int    correct_ = 0;
    int    total_preds_ = 0;
    int    per_class_pred_[cfg::NUM_CLASSES] = {};
    int    per_class_correct_[cfg::NUM_CLASSES] = {};

    // Session timing
    uint64_t session_start_ns_ = 0;
};
