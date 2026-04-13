#include "telemetry.h"
#include "config.h"

#include <cstring>
#include <cmath>
#include <algorithm>
#include <time.h>

uint64_t Telemetry::now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

uint64_t Telemetry::elapsed_ns(uint64_t start) {
    return now_ns() - start;
}

Telemetry::Telemetry(const char* csv_path) {
    std::memset(ring_, 0, sizeof(ring_));
    std::memset(idx_, 0, sizeof(idx_));
    std::memset(count_, 0, sizeof(count_));
    start_ns_ = now_ns();

    if (csv_path) {
        csv_ = fopen(csv_path, "w");
        if (csv_) {
            fprintf(csv_,
                "timestamp_ms,"
                "predicted,confidence,ground_truth,"
                "motor_m1,motor_m2,motor_m3,"
                "rms_ch0,rms_ch1,rms_ch2,"
                "snapshot_us,preprocess_us,infer_us,traj_us,uart_us,total_us,"
                "state\n");
            printf("[Telemetry] CSV → %s\n", csv_path);
        }
    }
}

Telemetry::~Telemetry() {
    if (csv_) fclose(csv_);
}

void Telemetry::record(Stage s, uint64_t ns) {
    int si = static_cast<int>(s);
    ring_[si][idx_[si]] = ns;
    idx_[si] = (idx_[si] + 1) % WINDOW;
    if (count_[si] < WINDOW) count_[si]++;
}

void Telemetry::log_row(int predicted, float confidence, int ground_truth,
                         float m1, float m2, float m3,
                         const float rms[3], const char* state)
{
    if (!csv_) return;

    auto us = [&](Stage s) -> double {
        int si = static_cast<int>(s);
        if (count_[si] == 0) return 0.0;
        int last = (idx_[si] - 1 + WINDOW) % WINDOW;
        return ring_[si][last] / 1000.0;
    };

    double ts_ms = (now_ns() - start_ns_) / 1e6;

    fprintf(csv_,
        "%.1f,%d,%.4f,%d,"
        "%.5f,%.5f,%.5f,"
        "%.4f,%.4f,%.4f,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
        "%s\n",
        ts_ms, predicted, confidence, ground_truth,
        m1, m2, m3,
        rms ? rms[0] : 0.0f, rms ? rms[1] : 0.0f, rms ? rms[2] : 0.0f,
        us(Stage::SNAPSHOT), us(Stage::PREPROCESS), us(Stage::INFERENCE),
        us(Stage::TRAJECTORY), us(Stage::UART_WRITE), us(Stage::TOTAL),
        state);
}

void Telemetry::print_summary() {
    static const char* names[] = {
        "Snapshot", "Preprocess", "Inference", "Trajectory", "UART", "TOTAL"
    };

    printf("\n═══ Session Summary ═══════════════════════════════════\n");
    double total_s = (now_ns() - start_ns_) / 1e9;
    printf("  Duration: %.1f s\n", total_s);
    printf("  ── Latency (ms) ──\n");
    for (int s = 0; s < N_STAGES; ++s) {
        if (count_[s] == 0) continue;
        int n = count_[s];
        uint64_t sum = 0, mx = 0, mn = UINT64_MAX;
        for (int i = 0; i < n; ++i) {
            sum += ring_[s][i];
            if (ring_[s][i] > mx) mx = ring_[s][i];
            if (ring_[s][i] < mn) mn = ring_[s][i];
        }
        double mean_ms = (sum / (double)n) / 1e6;
        printf("  %-12s  avg=%7.2f  min=%7.2f  max=%7.2f\n",
               names[s], mean_ms, mn / 1e6, mx / 1e6);
    }
    printf("═══════════════════════════════════════════════════════\n\n");
}
