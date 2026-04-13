#pragma once

#include <atomic>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "config.h"

#ifdef SIMULATION_MODE
#include <new>
#else
#include <cuda_runtime.h>
#endif

/*
 * Lock-free single-producer / single-consumer ring buffer.
 *
 * Backing store is CUDA pinned memory (cudaHostAllocMapped) so the GPU can
 * read it directly via zero-copy.  In SIMULATION_MODE we fall back to plain
 * heap memory so the code compiles without CUDA.
 *
 * Producer: SPI thread calls push() at 1 kHz.
 * Consumer: main thread calls snapshot() every 5 ms.
 */
class RingBuffer {
public:
    RingBuffer() {
        const size_t bytes = CAPACITY * cfg::NUM_CHANNELS * sizeof(float);
#ifdef SIMULATION_MODE
        data_ = static_cast<float*>(std::malloc(bytes));
#else
        cudaError_t err = cudaHostAlloc(&data_, bytes, cudaHostAllocMapped);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaHostAlloc failed: %s\n", cudaGetErrorString(err));
            std::abort();
        }
#endif
        std::memset(data_, 0, bytes);
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    ~RingBuffer() {
#ifdef SIMULATION_MODE
        std::free(data_);
#else
        cudaFreeHost(data_);
#endif
    }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    /* Producer: write one 3-channel sample. */
    void push(float ch0, float ch1, float ch2) {
        size_t h = head_.load(std::memory_order_relaxed);
        size_t slot = h & MASK;
        data_[slot * cfg::NUM_CHANNELS + 0] = ch0;
        data_[slot * cfg::NUM_CHANNELS + 1] = ch1;
        data_[slot * cfg::NUM_CHANNELS + 2] = ch2;
        head_.store(h + 1, std::memory_order_release);
    }

    /*
     * Consumer: copy the most recent WINDOW_SIZE samples into |dst|, laid out
     * as [CH0_t0 .. CH0_t204, CH1_t0 .. CH1_t204, CH2_t0 .. CH2_t204]
     * (channel-major, matching the [1,3,205] tensor layout for TensorRT).
     *
     * Returns the number of samples actually available (may be < WINDOW_SIZE
     * immediately after startup).
     */
    int snapshot(float* dst) const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t avail = h - tail_.load(std::memory_order_relaxed);
        if (avail > CAPACITY) avail = CAPACITY;

        int n = (avail >= static_cast<size_t>(cfg::WINDOW_SIZE))
                    ? cfg::WINDOW_SIZE
                    : static_cast<int>(avail);
        if (n == 0) return 0;

        size_t start = h - n;
        for (int i = 0; i < n; ++i) {
            size_t slot = (start + i) & MASK;
            for (int c = 0; c < cfg::NUM_CHANNELS; ++c)
                dst[c * cfg::WINDOW_SIZE + i] = data_[slot * cfg::NUM_CHANNELS + c];
        }
        return n;
    }

    size_t count() const {
        size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_relaxed);
        return h - t;
    }

private:
    static constexpr size_t CAPACITY = cfg::RING_CAPACITY;
    static constexpr size_t MASK     = CAPACITY - 1;
    static_assert((CAPACITY & MASK) == 0, "RING_CAPACITY must be power of 2");

    float* data_;
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};
