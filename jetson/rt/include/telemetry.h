#pragma once

#include <cstdint>
#include <cstdio>

enum class Stage {
    SNAPSHOT,
    PREPROCESS,
    INFERENCE,
    TRAJECTORY,
    UART_WRITE,
    TOTAL,
    NUM_STAGES
};

class Telemetry {
public:
    explicit Telemetry(const char* csv_path = nullptr);
    ~Telemetry();

    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;

    void record(Stage s, uint64_t ns);

    /* Research-grade CSV row with all descriptives. */
    void log_row(int predicted, float confidence, int ground_truth,
                 float m1, float m2, float m3,
                 const float rms[3], const char* state);

    void print_summary();

    static uint64_t now_ns();
    static uint64_t elapsed_ns(uint64_t start);

    bool csv_active() const { return csv_ != nullptr; }

private:
    static constexpr int N_STAGES = static_cast<int>(Stage::NUM_STAGES);
    static constexpr int WINDOW   = 1000;

    uint64_t ring_[N_STAGES][WINDOW] = {};
    int      idx_[N_STAGES] = {};
    int      count_[N_STAGES] = {};
    int      print_counter_ = 0;
    uint64_t start_ns_ = 0;
    FILE*    csv_ = nullptr;
};
