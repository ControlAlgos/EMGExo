#pragma once

// Uncomment or pass -DSIMULATION_MODE to cmake to enable synthetic EMG
// #define SIMULATION_MODE

#include <cstdint>

namespace cfg {

// --- SPI / ADC ---
constexpr const char* SPI_DEVICE    = "/dev/spidev0.0";
constexpr uint32_t    SPI_SPEED_HZ  = 1000000;   // 1 MHz
constexpr int         NUM_CHANNELS  = 3;
constexpr float       ADC_MAX       = 1023.0f;    // MCP3008 is 10-bit

// Channel mapping: CH0 = Ulnar/Radial, CH1 = Flex/Ext, CH2 = Pro/Sup
constexpr int CH_ULNAR_RADIAL = 0;
constexpr int CH_FLEX_EXT     = 1;
constexpr int CH_PRO_SUP      = 2;

// --- Sampling & Windowing ---
constexpr int SAMPLE_RATE_HZ   = 1000;
constexpr int WINDOW_SIZE      = 205;           // ~205 ms at 1 kHz
constexpr int INFER_INTERVAL_MS = 5;            // 5 ms control loop tick
constexpr int RING_CAPACITY    = 1024;          // power-of-2, > WINDOW_SIZE

// --- Model ---
constexpr int NUM_CLASSES = 7;
constexpr const char* INPUT_NAME  = "emg_input";
constexpr const char* OUTPUT_NAME = "gesture_logits";

// Gesture class indices (must match train.py label order)
constexpr int CLASS_REST           = 0;
constexpr int CLASS_WRIST_FLEX     = 1;
constexpr int CLASS_WRIST_EXT      = 2;
constexpr int CLASS_MASS_FLEX      = 3;
constexpr int CLASS_MASS_EXT       = 4;
constexpr int CLASS_PRONATION      = 5;
constexpr int CLASS_SUPINATION     = 6;

// --- Safety Thresholds ---
constexpr float RMS_THRESHOLD        = 0.05f;
constexpr float CONFIDENCE_THRESHOLD = 0.40f;
constexpr int   VOTE_WINDOW          = 60;  // 300ms of smoothing for stable display
constexpr float MAX_DELTA_RAD        = 0.01f;   // 2 rad/s — smooth but responsive
constexpr int   WATCHDOG_TIMEOUT_MS  = 500;

// --- Trajectory ---
constexpr float AMPLITUDE_SCALE = 0.30f;         // 30% amplitude for safe demo
constexpr int   TRANSITION_MS   = 500;            // 500 ms gentle blend
constexpr int   TRANSITION_TICKS = TRANSITION_MS / INFER_INTERVAL_MS;
constexpr float BLEND_STEP      = 1.0f / TRANSITION_TICKS;

// --- Auto-play ---
constexpr int   AUTOPLAY_GESTURE_MS = 12000;      // 12 s per gesture (~1.2 Fourier cycles)
constexpr int   AUTOPLAY_REST_MS    = 1000;        // 1 s REST between DoFs

// --- UART ---
constexpr const char* UART_DEVICE = "/dev/ttyTHS1";
constexpr int         UART_BAUD   = 921600;
constexpr uint8_t     START_BYTE  = 0xA5;

// --- Motor Defaults ---
constexpr int   MOTOR_IDS[3]  = {1, 2, 3};
constexpr float DEFAULT_KP    = 50.0f;
constexpr float DEFAULT_KD    = 1.5f;

// --- RT Scheduling ---
constexpr int SPI_THREAD_PRIORITY  = 99;        // SCHED_FIFO
constexpr int MAIN_THREAD_PRIORITY = 80;
constexpr int SPI_THREAD_CORE      = 1;
constexpr int MAIN_THREAD_CORE     = 0;

}  // namespace cfg
