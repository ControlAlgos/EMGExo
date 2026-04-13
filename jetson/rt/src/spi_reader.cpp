#include "spi_reader.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sched.h>
#include <pthread.h>
#include <thread>

#ifndef SIMULATION_MODE
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#endif

// ─── MCP3008 real hardware ──────────────────────────────────────────────

SpiReader::SpiReader(RingBuffer& ring) : ring_(ring) {
#ifndef SIMULATION_MODE
    fd_ = open(cfg::SPI_DEVICE, O_RDWR);
    if (fd_ < 0) {
        perror("SPI open");
        std::abort();
    }
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = cfg::SPI_SPEED_HZ;
    ioctl(fd_, SPI_IOC_WR_MODE, &mode);
    ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
#endif
}

SpiReader::~SpiReader() {
    stop();
#ifndef SIMULATION_MODE
    if (fd_ >= 0) close(fd_);
#endif
}

#ifndef SIMULATION_MODE
uint16_t SpiReader::read_channel(int ch) {
    uint8_t tx[3] = {0x01, static_cast<uint8_t>(0x80 | (ch << 4)), 0x00};
    uint8_t rx[3] = {0};

    struct spi_ioc_transfer xfer;
    std::memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = reinterpret_cast<unsigned long>(tx);
    xfer.rx_buf = reinterpret_cast<unsigned long>(rx);
    xfer.len = 3;
    xfer.speed_hz = cfg::SPI_SPEED_HZ;
    xfer.bits_per_word = 8;

    ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);
    return ((rx[1] & 0x03) << 8) | rx[2];
}
#else
uint16_t SpiReader::read_channel(int) { return 0; }
#endif

void SpiReader::start() {
    running.store(true, std::memory_order_relaxed);
    std::thread(&SpiReader::run, this).detach();
}

void SpiReader::stop() {
    running.store(false, std::memory_order_relaxed);
    usleep(5000);
}

void SpiReader::run() {
    // RT priority: SCHED_FIFO 99
    struct sched_param sp;
    sp.sched_priority = cfg::SPI_THREAD_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
        perror("sched_setscheduler SPI");

    // Pin to core 1
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cfg::SPI_THREAD_CORE, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    const long interval_ns = 1000000000L / cfg::SAMPLE_RATE_HZ;  // 1ms

#ifdef SIMULATION_MODE
    double sim_t = 0.0;
    const double sim_dt = 1.0 / cfg::SAMPLE_RATE_HZ;
#endif

    while (running.load(std::memory_order_relaxed)) {
        float ch[cfg::NUM_CHANNELS];

#ifdef SIMULATION_MODE
        ch[0] = 0.5f + 0.3f * sinf(2.0f * M_PI * 10.0f * sim_t);
        ch[1] = 0.5f + 0.3f * sinf(2.0f * M_PI * 25.0f * sim_t);
        ch[2] = 0.5f + 0.3f * sinf(2.0f * M_PI * 40.0f * sim_t);
        // Add some noise
        ch[0] += 0.02f * ((rand() % 200 - 100) / 100.0f);
        ch[1] += 0.02f * ((rand() % 200 - 100) / 100.0f);
        ch[2] += 0.02f * ((rand() % 200 - 100) / 100.0f);
        sim_t += sim_dt;
#else
        for (int c = 0; c < cfg::NUM_CHANNELS; ++c) {
            uint16_t raw = read_channel(c);
            ch[c] = static_cast<float>(raw & 0x3FF) / cfg::ADC_MAX;
        }
#endif

        ring_.push(ch[0], ch[1], ch[2]);

        // Absolute-time sleep to prevent drift
        next.tv_nsec += interval_ns;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec += 1;
            next.tv_nsec -= 1000000000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}
