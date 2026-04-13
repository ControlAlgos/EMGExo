#include "hud.h"
#include "config.h"
#include "telemetry.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

#define RST   "\033[0m"
#define BOLD  "\033[1m"
#define DIM   "\033[2m"
#define B_RED     "\033[1;91m"
#define B_GREEN   "\033[1;92m"
#define B_YELLOW  "\033[1;93m"
#define B_BLUE    "\033[1;94m"
#define B_MAGENTA "\033[1;95m"
#define B_CYAN    "\033[1;96m"
#define B_WHITE   "\033[1;97m"

#define TL "┌"
#define TR "┐"
#define BL "└"
#define BR "┘"
#define H  "─"
#define V  "│"
#define LT "├"
#define RT_ "┤"

static const int W = 72;
static const int BAR_W = 26;
static const float A_LO = -1.5f, A_HI = 1.5f;

static const char* NAMES[] = {
    "REST","WRIST_FLEX","WRIST_EXT","MASS_FLEX","MASS_EXT","PRONATION","SUPINATION",
};
static const char* COLORS[] = {
    DIM, B_GREEN, B_CYAN, B_YELLOW, B_MAGENTA, B_BLUE, B_RED,
};
static const char* SPARK[] = {" ","▁","▂","▃","▄","▅","▆","▇","█"};

const char* Hud::intent_name(int id) {
    if (id >= 0 && id < cfg::NUM_CLASSES) return NAMES[id];
    return "???";
}

static void hl(const char* l, const char* r) {
    printf("%s%s", B_WHITE, l);
    for (int i = 0; i < W; i++) printf(H);
    printf("%s" RST "\n", r);
}

static void pend(int used) {
    for (int i = used; i < W; i++) putchar(' ');
    printf(B_WHITE V RST "\n");
}

static void bar(char* o, float v, float lo, float hi) {
    float n = std::max(0.f, std::min(1.f, (v - lo) / (hi - lo)));
    int f = (int)(n * BAR_W);
    int c = (int)((0.f - lo) / (hi - lo) * BAR_W);
    c = std::max(0, std::min(BAR_W, c));
    for (int i = 0; i < BAR_W; i++) {
        if (f >= c && i >= c && i <= f) o[i] = '=';
        else if (f < c && i >= f && i < c) o[i] = '=';
        else if (i == c) o[i] = '|';
        else o[i] = ' ';
    }
    o[BAR_W] = 0;
}

Hud::Hud() { session_start_ns_ = Telemetry::now_ns(); }
Hud::~Hud() {}

double Hud::session_seconds() const {
    return (Telemetry::now_ns() - session_start_ns_) / 1e9;
}

void Hud::push_raw_emg(const float* buf) {
    for (int c = 0; c < cfg::NUM_CHANNELS; c++)
        std::memcpy(raw_window_[c], buf + c * cfg::WINDOW_SIZE,
                     cfg::WINDOW_SIZE * sizeof(float));
    raw_valid_ = true;
}

void Hud::push_emg_rms(const float rms[3]) {
    subsample_++;
    if (subsample_ < SUBSAMPLE_RATE) return;
    subsample_ = 0;
    for (int c = 0; c < 3; c++)
        emg_hist_[c][hist_write_] = rms[c];
    hist_write_ = (hist_write_ + 1) % SPARK_LEN;
    if (hist_count_ < SPARK_LEN) hist_count_++;
}

void Hud::record_classification(int gt, int pred) {
    if (gt < 0 || gt >= cfg::NUM_CLASSES) return;
    if (pred < 0 || pred >= cfg::NUM_CLASSES) return;
    total_preds_++;
    per_class_pred_[pred]++;
    if (pred == gt) { correct_++; per_class_correct_[gt]++; }
}

void Hud::record_latency(double loop_us) {
    double ms = loop_us / 1000.0;
    tick_count_++;
    sum_loop_ms_ += ms;
    avg_loop_ms_ = sum_loop_ms_ / tick_count_;
    if (ms > max_loop_ms_) max_loop_ms_ = ms;
    if (ms < min_loop_ms_ && ms > 0.001) min_loop_ms_ = ms;
}

void Hud::render(int intent, float confidence,
                 float m1, float m2, float m3,
                 double total_us, double /*infer_us*/,
                 const char* /*state_str*/,
                 int /*streamer_gesture*/,
                 const float rms[3],
                 double uart_us,
                 const char* mode_label,
                 bool autoplay_on,
                 int autoplay_gesture,
                 float autoplay_progress,
                 float autoplay_elapsed_s,
                 float autoplay_total_s)
{
    if (first_) { printf("\033[2J\033[?25l"); first_ = false; }
    printf("\033[H");

    const char* ic = (intent >= 0 && intent < cfg::NUM_CLASSES) ? COLORS[intent] : B_WHITE;

    // Row 1-2: Title
    hl(TL, TR);
    int ss = (int)session_seconds();
    float acc = total_preds_ > 0 ? 100.f * correct_ / total_preds_ : 0.f;
    printf(B_WHITE V RST "  " B_CYAN "EMGExo RT" RST);
    if (mode_label) printf(" " B_YELLOW "%s" RST, mode_label);
    printf("  Amp:" B_GREEN "%.0f%%" RST, cfg::AMPLITUDE_SCALE * 100.f);
    if (total_preds_ > 0)
        printf("  Acc:" B_GREEN "%.0f%%" RST, acc);
    printf("  %02d:%02d", ss / 60, ss % 60);
    pend(56);

    // Row 3-4: CNN + Schedule on one line
    hl(LT, RT_);
    printf(B_WHITE V RST "  CNN: %s" BOLD "%-14s" RST " %.2f",
           ic, intent_name(intent), confidence);
    if (autoplay_on) {
        const char* ac = (autoplay_gesture >= 0 && autoplay_gesture < cfg::NUM_CLASSES)
                         ? COLORS[autoplay_gesture] : DIM;
        printf("  " B_GREEN "▶" RST " %s%-10s" RST, ac, intent_name(autoplay_gesture));
        int bw = 12;
        int bf = (int)(autoplay_progress * bw);
        for (int i = 0; i < bw; i++) printf(i < bf ? "█" : "░");
        printf(" %.0fs", autoplay_total_s - autoplay_elapsed_s);
    } else {
        printf("  " DIM "⏸ MANUAL (a=auto)" RST);
    }
    pend(70);

    // Row 5-8: EMG Waveform (3 channels)
    hl(LT, RT_);
    const char* cn[] = {"Rad/Uln", "Flx/Ext", "Pro/Sup"};
    const char* cc[] = {B_CYAN, B_GREEN, B_BLUE};

    if (raw_valid_) {
        int step = cfg::WINDOW_SIZE / WAVE_DISP;
        for (int c = 0; c < 3; c++) {
            float vmin = 1e9f, vmax = -1e9f;
            for (int i = 0; i < cfg::WINDOW_SIZE; i++) {
                float v = raw_window_[c][i];
                if (v < vmin) vmin = v;
                if (v > vmax) vmax = v;
            }
            float range = vmax - vmin;
            if (range < 1e-7f) range = 1.f;

            printf(B_WHITE V RST " %s%-7s" RST " ", cc[c], cn[c]);
            for (int i = 0; i < WAVE_DISP; i++) {
                float sum = 0; int cnt = 0;
                int base = i * step;
                for (int j = 0; j < step && base+j < cfg::WINDOW_SIZE; j++)
                    { sum += raw_window_[c][base+j]; cnt++; }
                int lv = (int)((sum/cnt - vmin) / range * 8.f);
                lv = std::max(0, std::min(8, lv));
                printf("%s%s" RST, cc[c], SPARK[lv]);
            }
            float cr = rms ? rms[c] : 0.f;
            printf(" %5.3f", cr);
            pend(66);
        }
    }

    // Row 9-12: Joint angles + timing (combined)
    hl(LT, RT_);
    char b[BAR_W + 1];
    const char* jn[] = {"M1 Rad/Uln", "M2 Flx/Ext", "M3 Pro/Sup"};
    float ang[] = {m1, m2, m3};
    double e2e_ms = total_us / 1000.0;
    char timing_bufs[3][48];
    snprintf(timing_bufs[0], 48, "Sensor\u2192Motor: " B_GREEN "%.2f ms" RST, e2e_ms);
    snprintf(timing_bufs[1], 48, "Avg: %.2f ms  Max: %.2f ms", avg_loop_ms_, max_loop_ms_);
    snprintf(timing_bufs[2], 48, "Tick: %d", tick_count_);
    (void)uart_us;

    for (int j = 0; j < 3; j++) {
        bar(b, ang[j], A_LO, A_HI);
        const char* jc = (std::fabs(ang[j]) > 0.005f) ? B_CYAN : DIM;
        printf(B_WHITE V RST " %-10s %s%+6.3f" RST " [%s%s" RST "]  %s",
               jn[j], jc, ang[j], jc, b, timing_bufs[j]);
        pend(62);
    }

    // Row 13-14: Keys + bottom
    hl(LT, RT_);
    printf(B_WHITE V RST " " DIM
           "[1=Flex] [2=Pro] [3=Mass] [0=Rest]  [p] [a=Auto] [q=Quit]" RST);
    pend(62);
    hl(BL, BR);

    fflush(stdout);
}
