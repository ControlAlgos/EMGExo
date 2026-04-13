#include "trt_engine.h"
#include "config.h"

#include <fstream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef SIMULATION_MODE

// ─── Stub: no TensorRT, returns random class ────────────────────────────

struct TrtEngine::Impl {};

TrtEngine::TrtEngine(const std::string&) : impl_(nullptr) {
    input_buf = static_cast<float*>(std::malloc(
        cfg::NUM_CHANNELS * cfg::WINDOW_SIZE * sizeof(float)));
    std::memset(input_buf, 0,
        cfg::NUM_CHANNELS * cfg::WINDOW_SIZE * sizeof(float));
    printf("[TrtEngine] SIMULATION_MODE — no engine loaded\n");
}

TrtEngine::~TrtEngine() {
    std::free(input_buf);
}

InferResult TrtEngine::infer() {
    if (mock_gesture_) {
        int g = mock_gesture_->load(std::memory_order_relaxed);
        float c = mock_conf_ ? mock_conf_->load(std::memory_order_relaxed) : 0.99f;
        return {g, c};
    }
    return {cfg::CLASS_REST, 1.0f};
}

#else  // ── Real TensorRT ────────────────────────────────────────────────

#include <NvInfer.h>
#include <cuda_runtime.h>

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            fprintf(stderr, "[TRT] %s\n", msg);
    }
};

struct TrtEngine::Impl {
    Logger                          logger;
    nvinfer1::IRuntime*             runtime  = nullptr;
    nvinfer1::ICudaEngine*          engine   = nullptr;
    nvinfer1::IExecutionContext*    context  = nullptr;
    cudaStream_t                    stream   = nullptr;

    float* output_host = nullptr;   // pinned host output
    void*  d_input     = nullptr;   // device pointer for input (zero-copy)
    void*  d_output    = nullptr;   // device pointer for output (zero-copy)
};

TrtEngine::TrtEngine(const std::string& engine_path) : impl_(new Impl) {
    // Read serialized engine
    std::ifstream file(engine_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        fprintf(stderr, "Cannot open engine: %s\n", engine_path.c_str());
        std::abort();
    }
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> blob(size);
    file.read(blob.data(), size);

    impl_->runtime = nvinfer1::createInferRuntime(impl_->logger);
    impl_->engine  = impl_->runtime->deserializeCudaEngine(blob.data(), size);
    impl_->context = impl_->engine->createExecutionContext();
    cudaStreamCreate(&impl_->stream);

    // Zero-copy input buffer: GPU reads directly from pinned host
    size_t in_bytes = cfg::NUM_CHANNELS * cfg::WINDOW_SIZE * sizeof(float);
    cudaHostAlloc(&input_buf, in_bytes, cudaHostAllocMapped);
    std::memset(input_buf, 0, in_bytes);
    cudaHostGetDevicePointer(&impl_->d_input, input_buf, 0);

    // Zero-copy output buffer: GPU writes directly to pinned host
    size_t out_bytes = cfg::NUM_CLASSES * sizeof(float);
    cudaHostAlloc(reinterpret_cast<void**>(&impl_->output_host),
                  out_bytes, cudaHostAllocMapped);
    std::memset(impl_->output_host, 0, out_bytes);
    cudaHostGetDevicePointer(&impl_->d_output, impl_->output_host, 0);

    printf("[TrtEngine] Loaded %s  in=%zuB  out=%zuB\n",
           engine_path.c_str(), in_bytes, out_bytes);
}

TrtEngine::~TrtEngine() {
    if (impl_) {
        if (impl_->stream)  cudaStreamDestroy(impl_->stream);
        if (impl_->context) impl_->context->destroy();
        if (impl_->engine)  impl_->engine->destroy();
        if (impl_->runtime) impl_->runtime->destroy();
        if (input_buf)            cudaFreeHost(input_buf);
        if (impl_->output_host)   cudaFreeHost(impl_->output_host);
        delete impl_;
    }
}

InferResult TrtEngine::infer() {
    void* bindings[2] = { impl_->d_input, impl_->d_output };
    impl_->context->enqueueV2(bindings, impl_->stream, nullptr);
    cudaStreamSynchronize(impl_->stream);

    // Softmax + argmax on 7 logits (CPU, trivial cost)
    float* logits = impl_->output_host;
    float max_val = logits[0];
    for (int i = 1; i < cfg::NUM_CLASSES; ++i)
        if (logits[i] > max_val) max_val = logits[i];

    float sum = 0.0f;
    float probs[cfg::NUM_CLASSES];
    for (int i = 0; i < cfg::NUM_CLASSES; ++i) {
        probs[i] = std::exp(logits[i] - max_val);
        sum += probs[i];
    }

    int best = 0;
    float best_p = 0.0f;
    for (int i = 0; i < cfg::NUM_CLASSES; ++i) {
        probs[i] /= sum;
        if (probs[i] > best_p) {
            best_p = probs[i];
            best = i;
        }
    }
    return {best, best_p};
}

#endif  // SIMULATION_MODE
