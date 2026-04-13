#pragma once

#include "ring_buffer.h"
#include <atomic>
#include <string>
#include <vector>

struct PredEntry {
    uint8_t class_id;
    float   confidence;
};

struct AutoPhase {
    int gesture_id;
    int duration_ms;       // how long to stay in this phase
};

class DatasetStreamer {
public:
    DatasetStreamer(RingBuffer& ring, const std::string& bin_dir);
    ~DatasetStreamer();

    DatasetStreamer(const DatasetStreamer&) = delete;
    DatasetStreamer& operator=(const DatasetStreamer&) = delete;

    void start();
    void stop();

    std::atomic<bool> running{false};
    std::atomic<int>  active_gesture{0};   // current gesture being played
    std::atomic<bool> quit_requested{false};

    /* Pre-computed CNN prediction at the current playback position. */
    std::atomic<int>   cnn_pred_class{0};
    std::atomic<float> cnn_pred_conf{0.0f};
    std::atomic<bool>  has_predictions{false};

    /* Auto-play state (readable by HUD) */
    std::atomic<bool>  autoplay_active{true};
    std::atomic<int>   autoplay_phase_gesture{0};    // gesture ID in current phase
    std::atomic<float> autoplay_phase_progress{0.0f}; // 0.0 → 1.0 within phase
    std::atomic<float> autoplay_phase_elapsed_s{0.0f};
    std::atomic<float> autoplay_phase_total_s{0.0f};

    const std::vector<AutoPhase>& schedule() const { return schedule_; }

private:
    void run_producer();
    void run_keyboard();

    bool load_bin(int class_id, const std::string& path);
    bool load_pred(int class_id, const std::string& path);

    void build_schedule();

    RingBuffer& ring_;
    std::string bin_dir_;

    struct GestureData {
        std::vector<float>     samples;       // interleaved [sample][3]
        int                    num_samples = 0;
        std::vector<PredEntry> predictions;
        int                    pred_stride = 100;
    };

    GestureData gestures_[7];
    bool        loaded_[7] = {};

    std::vector<AutoPhase> schedule_;
};
