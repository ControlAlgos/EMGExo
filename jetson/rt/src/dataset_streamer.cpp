#include "dataset_streamer.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <thread>
#include <time.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <pthread.h>

static const char* CLASS_FILES[] = {
    "rest.bin",
    "wrist_flex.bin",
    "wrist_ext.bin",
    "mass_flex.bin",
    "mass_ext.bin",
    "pronation.bin",
    "supination.bin",
};

static const char* PRED_FILES[] = {
    "rest.pred.bin",
    "wrist_flex.pred.bin",
    "wrist_ext.pred.bin",
    "mass_flex.pred.bin",
    "mass_ext.pred.bin",
    "pronation.pred.bin",
    "supination.pred.bin",
};

static const char* CLASS_NAMES[] = {
    "REST", "WRIST_FLEX", "WRIST_EXT", "MASS_FLEX",
    "MASS_EXT", "PRONATION", "SUPINATION",
};

// ─── Load helpers ───────────────────────────────────────────────────────

bool DatasetStreamer::load_bin(int class_id, const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return false;

    size_t bytes = f.tellg();
    f.seekg(0, std::ios::beg);

    size_t num_floats = bytes / sizeof(float);
    if (num_floats < 3 || num_floats % 3 != 0) {
        fprintf(stderr, "[DatasetStreamer] %s: bad size (%zu bytes)\n",
                path.c_str(), bytes);
        return false;
    }

    gestures_[class_id].samples.resize(num_floats);
    f.read(reinterpret_cast<char*>(gestures_[class_id].samples.data()), bytes);
    gestures_[class_id].num_samples = static_cast<int>(num_floats / 3);
    loaded_[class_id] = true;

    printf("[DatasetStreamer] Loaded class %d (%s): %s (%d samples, %.1f s)\n",
           class_id, CLASS_NAMES[class_id], path.c_str(),
           gestures_[class_id].num_samples,
           gestures_[class_id].num_samples / (float)cfg::SAMPLE_RATE_HZ);
    return true;
}

bool DatasetStreamer::load_pred(int class_id, const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) return false;

    size_t bytes = f.tellg();
    f.seekg(0, std::ios::beg);

    constexpr size_t ENTRY_SIZE = sizeof(uint8_t) + sizeof(float);
    size_t n_entries = bytes / ENTRY_SIZE;
    if (n_entries == 0) return false;

    gestures_[class_id].predictions.resize(n_entries);

    for (size_t i = 0; i < n_entries; ++i) {
        uint8_t cls;
        float conf;
        f.read(reinterpret_cast<char*>(&cls), sizeof(cls));
        f.read(reinterpret_cast<char*>(&conf), sizeof(conf));
        gestures_[class_id].predictions[i] = {cls, conf};
    }

    printf("[DatasetStreamer] Loaded predictions for class %d: %zu entries\n",
           class_id, n_entries);
    return true;
}

void DatasetStreamer::build_schedule() {
    schedule_.clear();

    // Structured DoF-by-DoF demonstration
    struct DofGroup {
        const char* label;
        int gestures[2];   // forward / reverse (or -1 if absent)
    };
    DofGroup dofs[] = {
        {"Flex/Ext (M2)",  {cfg::CLASS_WRIST_FLEX, cfg::CLASS_WRIST_EXT}},
        {"Pro/Sup  (M3)",  {cfg::CLASS_PRONATION,  cfg::CLASS_SUPINATION}},
        {"Combined (M1+2)",{cfg::CLASS_MASS_FLEX,   cfg::CLASS_MASS_EXT}},
    };

    bool first_group = true;
    for (auto& dof : dofs) {
        bool any = false;
        for (int g : dof.gestures) {
            if (g >= 0 && g < cfg::NUM_CLASSES && loaded_[g]) any = true;
        }
        if (!any) continue;

        // Skip REST before the very first DoF group so motors move immediately
        if (!first_group)
            schedule_.push_back({cfg::CLASS_REST, cfg::AUTOPLAY_REST_MS});
        first_group = false;

        for (int g : dof.gestures) {
            if (g >= 0 && g < cfg::NUM_CLASSES && loaded_[g])
                schedule_.push_back({g, cfg::AUTOPLAY_GESTURE_MS});
        }
    }

    if (schedule_.empty())
        schedule_.push_back({cfg::CLASS_REST, 5000});

    printf("[DatasetStreamer] Auto-play schedule (%zu phases):\n", schedule_.size());
    int phase = 0;
    for (auto& dof : dofs) {
        printf("  DoF: %s\n", dof.label);
    }
    for (size_t i = 0; i < schedule_.size(); ++i) {
        printf("  [%zu] %-14s  %.1f s\n", i,
               CLASS_NAMES[schedule_[i].gesture_id],
               schedule_[i].duration_ms / 1000.0f);
    }
    (void)phase;
}

DatasetStreamer::DatasetStreamer(RingBuffer& ring, const std::string& bin_dir)
    : ring_(ring), bin_dir_(bin_dir)
{
    std::memset(loaded_, 0, sizeof(loaded_));

    int loaded_count = 0;
    int pred_count = 0;
    for (int i = 0; i < cfg::NUM_CLASSES; ++i) {
        std::string path = bin_dir_ + "/" + CLASS_FILES[i];
        if (load_bin(i, path)) {
            loaded_count++;
            std::string pred_path = bin_dir_ + "/" + PRED_FILES[i];
            if (load_pred(i, pred_path))
                pred_count++;
        }
    }

    if (loaded_count == 0) {
        fprintf(stderr, "[DatasetStreamer] No .bin files found in %s\n",
                bin_dir_.c_str());
        std::abort();
    }

    if (!loaded_[cfg::CLASS_REST]) {
        int n = cfg::SAMPLE_RATE_HZ * 5;
        gestures_[cfg::CLASS_REST].samples.resize(n * 3, 0.0f);
        gestures_[cfg::CLASS_REST].num_samples = n;
        loaded_[cfg::CLASS_REST] = true;
        printf("[DatasetStreamer] Synthesized 5s of REST silence\n");
    }

    for (int i = 0; i < cfg::NUM_CLASSES; ++i) {
        auto& gd = gestures_[i];
        if (!gd.predictions.empty() && gd.num_samples > 0) {
            gd.pred_stride = gd.num_samples / static_cast<int>(gd.predictions.size());
            if (gd.pred_stride < 1) gd.pred_stride = 1;
        }
    }

    has_predictions.store(pred_count > 0, std::memory_order_relaxed);
    build_schedule();

    printf("[DatasetStreamer] %d/%d classes loaded, %d with CNN predictions\n",
           loaded_count, cfg::NUM_CLASSES, pred_count);
}

DatasetStreamer::~DatasetStreamer() {
    stop();
}

void DatasetStreamer::start() {
    running.store(true, std::memory_order_relaxed);
    std::thread(&DatasetStreamer::run_producer, this).detach();
    std::thread(&DatasetStreamer::run_keyboard, this).detach();
}

void DatasetStreamer::stop() {
    running.store(false, std::memory_order_relaxed);
    usleep(5000);
}

// ─── Producer: push 3 floats at 1 kHz ──────────────────────────────────

void DatasetStreamer::run_producer() {
    struct sched_param sp;
    sp.sched_priority = cfg::SPI_THREAD_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        perror("[DatasetStreamer] sched_setscheduler");

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cfg::SPI_THREAD_CORE, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long interval_ns = 1000000000L / cfg::SAMPLE_RATE_HZ;

    int cursor = 0;
    int prev_gesture = -1;

    while (running.load(std::memory_order_relaxed)) {
        int g = active_gesture.load(std::memory_order_relaxed);

        if (g < 0 || g >= cfg::NUM_CLASSES || !loaded_[g])
            g = cfg::CLASS_REST;

        if (g != prev_gesture) {
            cursor = 0;
            prev_gesture = g;
        }

        const auto& gd = gestures_[g];
        int sample_pos = cursor % gd.num_samples;
        int idx = sample_pos * 3;
        ring_.push(gd.samples[idx], gd.samples[idx + 1], gd.samples[idx + 2]);

        if (!gd.predictions.empty()) {
            int pred_idx = sample_pos / gd.pred_stride;
            if (pred_idx >= static_cast<int>(gd.predictions.size()))
                pred_idx = static_cast<int>(gd.predictions.size()) - 1;
            const auto& pe = gd.predictions[pred_idx];
            cnn_pred_class.store(pe.class_id, std::memory_order_relaxed);
            cnn_pred_conf.store(pe.confidence, std::memory_order_relaxed);
        }

        cursor++;

        next.tv_nsec += interval_ns;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec += 1;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}

// ─── Keyboard + Auto-play scheduler ─────────────────────────────────────

void DatasetStreamer::run_keyboard() {
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 1;   // 100ms timeout per read()
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // Drain ALL stdin for 2 seconds to discard stale bytes from launcher
    tcflush(STDIN_FILENO, TCIFLUSH);
    for (int drain = 0; drain < 20; drain++) {
        char dummy;
        (void)read(STDIN_FILENO, &dummy, 1);
    }
    tcflush(STDIN_FILENO, TCIFLUSH);

    int sched_idx = 0;
    int sched_elapsed_ms = 0;
    const int tick_ms = 100;   // ~100ms per loop iteration (VTIME=1)

    while (running.load(std::memory_order_relaxed)) {
        char ch = 0;
        int n = read(STDIN_FILENO, &ch, 1);

        if (n > 0) {
            switch (ch) {
                case 'q': case 'Q': case 3:
                    quit_requested.store(true, std::memory_order_relaxed);
                    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
                    return;
                case 'p': case 'P': {
                    bool ap = autoplay_active.load(std::memory_order_relaxed);
                    autoplay_active.store(!ap, std::memory_order_relaxed);
                    break;
                }
                case 'a': case 'A':
                    autoplay_active.store(true, std::memory_order_relaxed);
                    break;
                case '1':
                    autoplay_active.store(false, std::memory_order_relaxed);
                    active_gesture.store(cfg::CLASS_WRIST_FLEX, std::memory_order_relaxed);
                    break;
                case '2':
                    autoplay_active.store(false, std::memory_order_relaxed);
                    active_gesture.store(cfg::CLASS_PRONATION, std::memory_order_relaxed);
                    break;
                case '3':
                    autoplay_active.store(false, std::memory_order_relaxed);
                    active_gesture.store(cfg::CLASS_MASS_FLEX, std::memory_order_relaxed);
                    break;
                case '0': case 'r': case 'R':
                    autoplay_active.store(false, std::memory_order_relaxed);
                    active_gesture.store(cfg::CLASS_REST, std::memory_order_relaxed);
                    break;
                default:
                    break;
            }
        }

        // Drive auto-play schedule
        if (autoplay_active.load(std::memory_order_relaxed) && !schedule_.empty()) {
            const auto& phase = schedule_[sched_idx];
            active_gesture.store(phase.gesture_id, std::memory_order_relaxed);

            autoplay_phase_gesture.store(phase.gesture_id, std::memory_order_relaxed);
            autoplay_phase_elapsed_s.store(sched_elapsed_ms / 1000.0f, std::memory_order_relaxed);
            autoplay_phase_total_s.store(phase.duration_ms / 1000.0f, std::memory_order_relaxed);
            autoplay_phase_progress.store(
                static_cast<float>(sched_elapsed_ms) / phase.duration_ms,
                std::memory_order_relaxed);

            sched_elapsed_ms += tick_ms;
            if (sched_elapsed_ms >= phase.duration_ms) {
                sched_elapsed_ms = 0;
                sched_idx = (sched_idx + 1) % static_cast<int>(schedule_.size());
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}
