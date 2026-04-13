cmake_minimum_required(VERSION 3.18)
project(emgexo_rt LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Options ──────────────────────────────────────────────────────────────
option(SIM "Enable simulation mode (no SPI, no TensorRT)" OFF)
option(LAPTOP_SIM "Laptop SIL: no CUDA, no TRT, UART console fallback" OFF)
if(LAPTOP_SIM)
    set(SIM ON CACHE BOOL "" FORCE)
endif()

# ── Sources ──────────────────────────────────────────────────────────────
add_executable(emgexo_rt
    src/main.cpp
    src/spi_reader.cpp
    src/dataset_streamer.cpp
    src/trt_engine.cpp
    src/trajectory.cpp
    src/uart_writer.cpp
    src/telemetry.cpp
    src/hud.cpp
)

target_include_directories(emgexo_rt PRIVATE include)
target_compile_options(emgexo_rt PRIVATE -O2 -Wall -Wextra)

if(SIM)
    target_compile_definitions(emgexo_rt PRIVATE SIMULATION_MODE)
    target_link_libraries(emgexo_rt PRIVATE pthread m)
else()
    # ── CUDA ─────────────────────────────────────────────────────────────
    find_package(CUDA REQUIRED)
    target_include_directories(emgexo_rt PRIVATE ${CUDA_INCLUDE_DIRS})

    # ── TensorRT ─────────────────────────────────────────────────────────
    find_library(NVINFER nvinfer HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)
    find_library(NVINFER_PLUGIN nvinfer_plugin HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu)

    if(NOT NVINFER)
        message(FATAL_ERROR "libnvinfer not found. Install TensorRT or build with -DSIM=ON")
    endif()

    target_link_libraries(emgexo_rt PRIVATE
        ${NVINFER}
        ${NVINFER_PLUGIN}
        ${CUDA_LIBRARIES}
        pthread m
    )
endif()
