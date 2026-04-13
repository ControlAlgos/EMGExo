#pragma once

#include <atomic>
#include <string>

struct InferResult {
    int   class_id;
    float confidence;
};

class TrtEngine {
public:
    explicit TrtEngine(const std::string& engine_path);
    ~TrtEngine();

    TrtEngine(const TrtEngine&) = delete;
    TrtEngine& operator=(const TrtEngine&) = delete;

    InferResult infer();

    /* In SIMULATION_MODE, infer() reads from these atomics to return
     * pre-computed CNN predictions from the DatasetStreamer. */
    void set_mock_gesture(std::atomic<int>* p) { mock_gesture_ = p; }
    void set_mock_confidence(std::atomic<float>* p) { mock_conf_ = p; }

    /* Direct pointer to the pinned input buffer [3 * WINDOW_SIZE floats].
     * The ring buffer snapshot() writes directly here — zero copy. */
    float* input_buf = nullptr;

private:
    struct Impl;
    Impl* impl_;
    std::atomic<int>*   mock_gesture_ = nullptr;
    std::atomic<float>* mock_conf_    = nullptr;
};
