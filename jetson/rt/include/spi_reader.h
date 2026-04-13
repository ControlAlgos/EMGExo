#pragma once

#include "ring_buffer.h"
#include <atomic>

class SpiReader {
public:
    explicit SpiReader(RingBuffer& ring);
    ~SpiReader();

    SpiReader(const SpiReader&) = delete;
    SpiReader& operator=(const SpiReader&) = delete;

    void start();                   // spawn capture thread
    void stop();                    // signal and join
    std::atomic<bool> running{false};

private:
    void run();                     // thread entry (SCHED_FIFO 99, core 1)
    uint16_t read_channel(int ch);  // single MCP3008 SPI read

    RingBuffer& ring_;
    int fd_ = -1;
};
