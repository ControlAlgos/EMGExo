#include "config.h"
#include "ring_buffer.h"
#include "spi_reader.h"
#include "dataset_streamer.h"
#include "trt_engine.h"
#include "trajectory.h"
#include "uart_writer.h"
#include "telemetry.h"
#include "hud.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <atomic>
#include <thread>
#include <getopt.h>
#include <sched.h>
#include <pthread.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

// ─── Globals ────────────────────────────────────────────────────────────

static volatile sig_atomic_t g_running = 1;
static void sig_handler(int) { g_running = 0; }

// ─── Majority voter ─────────────────────────────────────────────────────

class MajorityVoter {
public:
    explicit MajorityVoter(int window) : window_(window) {
        buf_ = new int[window];
        std::memset(buf_, 0, window * sizeof(int));
    }
    ~MajorityVoter() { delete[] buf_; }

    int vote(int cls) {
        buf_[idx_] = cls;
        idx_ = (idx_ + 1) % window_;
        if (count_ < window_) count_++;

        int freq[cfg::NUM_CLASSES] = {};
        for (int i = 0; i < count_; ++i)
            freq[buf_[i]]++;
        int best = 0;
        for (int i = 1; i < cfg::NUM_CLASSES; ++i)
            if (freq[i] > freq[best]) best = i;
        return best;
    }
private:
    int* buf_;
    int  window_;
    int  idx_   = 0;
    int  count_ = 0;
};

// ─── Preprocessing ──────────────────────────────────────────────────────

static float compute_rms(const float* buf, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) sum += buf[i] * buf[i];
    return std::sqrt(sum / n);
}

static void normalize_channel(float* ch, int n) {
    float mean = 0.0f;
    for (int i = 0; i < n; ++i) mean += ch[i];
    mean /= n;

    float var = 0.0f;
    for (int i = 0; i < n; ++i) {
        ch[i] -= mean;
        var += ch[i] * ch[i];
    }
    float sd = std::sqrt(var / n);
    if (sd < 1e-7f) sd = 1e-7f;
    for (int i = 0; i < n; ++i) ch[i] /= sd;
}

// ─── Standalone keyboard thread for sim mode ───────────────────────────

static std::atomic<int>  g_kb_gesture{0};
static std::atomic<bool> g_kb_quit{false};

static void keyboard_thread_fn() {
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    tcflush(STDIN_FILENO, TCIFLUSH);
    for (int drain = 0; drain < 20; drain++) {
        char dummy;
        (void)read(STDIN_FILENO, &dummy, 1);
    }
    tcflush(STDIN_FILENO, TCIFLUSH);

    while (!g_kb_quit.load(std::memory_order_relaxed)) {
        char ch = 0;
        if (read(STDIN_FILENO, &ch, 1) <= 0) continue;
        switch (ch) {
            case '1': g_kb_gesture.store(cfg::CLASS_WRIST_FLEX, std::memory_order_relaxed); break;
            case '2': g_kb_gesture.store(cfg::CLASS_PRONATION,  std::memory_order_relaxed); break;
            case '3': g_kb_gesture.store(cfg::CLASS_MASS_FLEX,  std::memory_order_relaxed); break;
            case '0': case 'r': case 'R':
                g_kb_gesture.store(cfg::CLASS_REST, std::memory_order_relaxed); break;
            case 'q': case 'Q': case 3:
                g_kb_quit.store(true, std::memory_order_relaxed); break;
            default: break;
        }
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

// ─── State name helper ──────────────────────────────────────────────────

static const char* state_name(int voted) {
    return (voted == cfg::CLASS_REST) ? "REST" : "ACT";
}

// ─── Usage / args ───────────────────────────────────────────────────────

static void usage(const char* prog) {
    printf("Usage: %s [options]\n"
           "  --engine  PATH   TensorRT engine file (required unless --sim)\n"
           "  --uart    PATH   UART device [%s]\n"
           "  --dataset PATH   Directory with .bin EMG files (dataset mode)\n"
           "  --mode    NAME   Mode label for HUD (e.g. 'Training', 'Rehab')\n"
           "  --csv     PATH   Telemetry CSV output\n"
           "  --sim            Simulation mode (synthetic EMG, no TRT)\n"
           "  --no-reset       Skip ESP32 boot wait\n"
           "  -h, --help       Show this help\n",
           prog, cfg::UART_DEVICE);
}

// ─── Main ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const char* engine_path  = nullptr;
    const char* uart_path    = cfg::UART_DEVICE;
    const char* csv_path     = nullptr;
    const char* dataset_path = nullptr;
    const char* mode_label   = nullptr;
    bool sim_mode   = false;
    bool do_reset   = true;

#ifdef SIMULATION_MODE
    sim_mode = true;
#endif

    static struct option long_opts[] = {
        {"engine",   required_argument, nullptr, 'e'},
        {"uart",     required_argument, nullptr, 'u'},
        {"dataset",  required_argument, nullptr, 'd'},
        {"mode",     required_argument, nullptr, 'm'},
        {"csv",      required_argument, nullptr, 'c'},
        {"sim",      no_argument,       nullptr, 'S'},
        {"no-reset", no_argument,       nullptr, 'R'},
        {"help",     no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "e:u:d:m:c:Sh", long_opts, nullptr)) != -1) {
        switch (opt) {
            case 'e': engine_path  = optarg; break;
            case 'u': uart_path    = optarg; break;
            case 'd': dataset_path = optarg; break;
            case 'm': mode_label   = optarg; break;
            case 'c': csv_path     = optarg; break;
            case 'S': sim_mode     = true;   break;
            case 'R': do_reset     = false;  break;
            case 'h': usage(argv[0]); return 0;
            default:  usage(argv[0]); return 1;
        }
    }

    bool dataset_mode = (dataset_path != nullptr);

    if (!sim_mode && !dataset_mode && !engine_path) {
        fprintf(stderr, "Error: --engine required (or use --sim / --dataset)\n");
        return 1;
    }

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // RT priority for main thread: SCHED_FIFO 80, core 0
    {
        struct sched_param sp;
        sp.sched_priority = cfg::MAIN_THREAD_PRIORITY;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
            perror("[main] sched_setscheduler (run as root?)");

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cfg::MAIN_THREAD_CORE, &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    }

    // Auto-generate CSV filename in dataset mode
    char auto_csv[256] = {};
    if (dataset_mode && !csv_path) {
        time_t now_t = time(nullptr);
        struct tm* lt = localtime(&now_t);
        snprintf(auto_csv, sizeof(auto_csv),
                 "emgexo_%04d%02d%02d_%02d%02d%02d.csv",
                 lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
                 lt->tm_hour, lt->tm_min, lt->tm_sec);
        csv_path = auto_csv;
    }

    const char* mode_str = dataset_mode ? "DATASET" : (sim_mode ? "SIMULATION" : "LIVE");
    printf("═══ EMGExo RT Control System ═══\n");
    printf("  Mode:       %s\n", mode_str);
    printf("  Engine:     %s\n", engine_path ? engine_path : "(sim/stub)");
    printf("  UART:       %s\n", uart_path);
    if (dataset_path) printf("  Dataset:    %s\n", dataset_path);
    if (csv_path)     printf("  CSV Log:    %s\n", csv_path);
    printf("  Amplitude:  %.0f%%\n", cfg::AMPLITUDE_SCALE * 100.0f);
    printf("  Δ-clamp:    %.4f rad/tick\n", cfg::MAX_DELTA_RAD);
    printf("  Blend:      %d ms\n", cfg::TRANSITION_MS);
    printf("  Interval:   %d ms\n", cfg::INFER_INTERVAL_MS);

    // ── Instantiate components ──────────────────────────────────────────

    RingBuffer ring;

    TrtEngine engine((sim_mode || dataset_mode) ? "" : engine_path);

    UartWriter uart(uart_path);
    if (do_reset && !dataset_mode && !uart.is_console_mode())
        uart.resetESP32(10);

    DatasetStreamer* ds = nullptr;
    SpiReader*       spi = nullptr;
    std::thread kb_thread;

    if (dataset_mode) {
        ds = new DatasetStreamer(ring, dataset_path);
        if (ds->has_predictions.load()) {
            engine.set_mock_gesture(&ds->cnn_pred_class);
            engine.set_mock_confidence(&ds->cnn_pred_conf);
        } else {
            engine.set_mock_gesture(&ds->active_gesture);
        }
        ds->start();
    } else {
        spi = new SpiReader(ring);
        spi->start();
        if (sim_mode) {
            engine.set_mock_gesture(&g_kb_gesture);
            kb_thread = std::thread(keyboard_thread_fn);
        }
    }

    TrajectoryGenerator traj;
    MajorityVoter voter(cfg::VOTE_WINDOW);
    Telemetry tel(csv_path);
    Hud hud;

    // Temp buffer to capture raw EMG before normalization
    float raw_emg_copy[cfg::NUM_CHANNELS * cfg::WINDOW_SIZE];

    printf("[main] Control loop starting...\n");

    // ── 5 ms control loop with absolute-time sleep ──────────────────────

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);

    int watchdog_counter = 0;
    const int watchdog_limit = cfg::WATCHDOG_TIMEOUT_MS / cfg::INFER_INTERVAL_MS;

    while (g_running) {
        // Check for quit from keyboard thread
        if (ds && ds->quit_requested.load(std::memory_order_relaxed))
            break;
        if (g_kb_quit.load(std::memory_order_relaxed))
            break;

        // Advance to next 5 ms tick
        next_tick.tv_nsec += cfg::INFER_INTERVAL_MS * 1000000L;
        if (next_tick.tv_nsec >= 1000000000L) {
            next_tick.tv_sec  += 1;
            next_tick.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_tick, nullptr);

        uint64_t t0 = Telemetry::now_ns();

        // 1. Snapshot ring buffer -> TRT input buffer
        int samples = ring.snapshot(engine.input_buf);
        uint64_t t1 = Telemetry::now_ns();
        tel.record(Stage::SNAPSHOT, t1 - t0);

        if (samples < cfg::WINDOW_SIZE) {
            watchdog_counter++;
            continue;
        }

        // Save raw EMG copy BEFORE normalization (for oscilloscope display)
        std::memcpy(raw_emg_copy, engine.input_buf, sizeof(raw_emg_copy));

        // Per-channel RMS (from raw, pre-normalization data)
        float ch_rms[3];
        for (int c = 0; c < cfg::NUM_CHANNELS; ++c)
            ch_rms[c] = compute_rms(raw_emg_copy + c * cfg::WINDOW_SIZE,
                                    cfg::WINDOW_SIZE);

        // 2. Preprocess: DC removal + Z-score, per channel
        for (int c = 0; c < cfg::NUM_CHANNELS; ++c)
            normalize_channel(engine.input_buf + c * cfg::WINDOW_SIZE,
                              cfg::WINDOW_SIZE);
        uint64_t t2 = Telemetry::now_ns();
        tel.record(Stage::PREPROCESS, t2 - t1);

        // 3. RMS gate + Inference
        float rms = compute_rms(engine.input_buf,
                                cfg::NUM_CHANNELS * cfg::WINDOW_SIZE);
        int intent;
        float confidence;

        if (rms < cfg::RMS_THRESHOLD) {
            intent = cfg::CLASS_REST;
            confidence = 1.0f;
        } else {
            InferResult res = engine.infer();
            intent     = res.class_id;
            confidence = res.confidence;
            if (confidence < cfg::CONFIDENCE_THRESHOLD)
                intent = cfg::CLASS_REST;
        }
        uint64_t t3 = Telemetry::now_ns();
        tel.record(Stage::INFERENCE, t3 - t2);

        if (intent != cfg::CLASS_REST)
            watchdog_counter = 0;
        else if (++watchdog_counter > watchdog_limit)
            intent = cfg::CLASS_REST;

        // 4. Majority vote
        int voted = voter.vote(intent);

        // In autoplay mode, the schedule drives trajectory (smooth, continuous).
        // CNN prediction is shown in HUD for accuracy analysis only.
        int traj_driver = voted;
        if (ds && ds->autoplay_active.load(std::memory_order_relaxed)) {
            int sched_g = ds->autoplay_phase_gesture.load(std::memory_order_relaxed);
            if (sched_g >= 0 && sched_g < cfg::NUM_CLASSES)
                traj_driver = sched_g;
        }
        traj.update(traj_driver);
        float m1, m2, m3;
        traj.compute(m1, m2, m3);
        uint64_t t4 = Telemetry::now_ns();
        tel.record(Stage::TRAJECTORY, t4 - t3);

        // 5. Fire-and-forget UART write
        uart.send_all(m1, m2, m3, cfg::DEFAULT_KP, cfg::DEFAULT_KD);
        uint64_t t5 = Telemetry::now_ns();
        tel.record(Stage::UART_WRITE, t5 - t4);

        uint64_t total_ns = t5 - t0;
        uint64_t infer_ns = t3 - t2;
        uint64_t uart_ns  = t5 - t4;
        tel.record(Stage::TOTAL, total_ns);

        // Ground truth from auto-play schedule (for accuracy + CSV)
        int ground_truth = -1;
        int streamer_g = -1;
        if (ds) {
            streamer_g  = ds->active_gesture.load(std::memory_order_relaxed);
            ground_truth = streamer_g;
        } else if (sim_mode) {
            streamer_g   = g_kb_gesture.load(std::memory_order_relaxed);
            ground_truth = streamer_g;
        }

        // Research CSV row
        tel.log_row(voted, confidence, ground_truth,
                    m1, m2, m3, ch_rms, state_name(voted));

        // HUD data
        bool ap_on = false;
        int  ap_gesture = 0;
        float ap_progress = 0.0f, ap_elapsed = 0.0f, ap_total = 0.0f;
        if (ds) {
            ap_on       = ds->autoplay_active.load(std::memory_order_relaxed);
            ap_gesture  = ds->autoplay_phase_gesture.load(std::memory_order_relaxed);
            ap_progress = ds->autoplay_phase_progress.load(std::memory_order_relaxed);
            ap_elapsed  = ds->autoplay_phase_elapsed_s.load(std::memory_order_relaxed);
            ap_total    = ds->autoplay_phase_total_s.load(std::memory_order_relaxed);
        }

        hud.push_raw_emg(raw_emg_copy);
        hud.push_emg_rms(ch_rms);
        hud.record_latency(total_ns / 1000.0);
        if (ground_truth >= 0)
            hud.record_classification(ground_truth, voted);

        // In autoplay, display the scheduled gesture (stable 12s) rather than noisy voter
        int display_intent = voted;
        float display_conf = confidence;
        if (ap_on && ap_gesture >= 0 && ap_gesture < cfg::NUM_CLASSES) {
            display_intent = ap_gesture;
            display_conf = 0.99f;
        }

        hud.render(display_intent, display_conf, m1, m2, m3,
                   total_ns / 1000.0, infer_ns / 1000.0,
                   state_name(voted), streamer_g,
                   ch_rms, uart_ns / 1000.0, mode_label,
                   ap_on, ap_gesture, ap_progress, ap_elapsed, ap_total);
    }

    // Restore cursor and clear below HUD
    printf("\033[?25h\033[35;0H\n");

    g_kb_quit.store(true, std::memory_order_relaxed);
    if (kb_thread.joinable()) kb_thread.join();

    if (ds)  ds->stop();
    if (spi) spi->stop();

    // Send REST position hold to all motors before exit
    {
        float m1, m2, m3;
        traj.update(cfg::CLASS_REST);
        traj.compute(m1, m2, m3);
        uart.send_all(m1, m2, m3, cfg::DEFAULT_KP, cfg::DEFAULT_KD);
    }

    tel.print_summary();

    if (csv_path) {
        printf("[main] CSV data saved → %s\n", csv_path);

        // Generate joint position/torque plots
        char plot_cmd[512];
        snprintf(plot_cmd, sizeof(plot_cmd),
                 "python3 \"%s/jetson/tools/plot_session.py\" \"%s\" 2>/dev/null &",
                 getenv("EMGEXO_ROOT") ? getenv("EMGEXO_ROOT") : ".",
                 csv_path);
        if (system(plot_cmd) == 0)
            printf("[main] Plot generation launched.\n");
    }

    delete ds;
    delete spi;

    printf("[main] Done.\n");
    return 0;
}
