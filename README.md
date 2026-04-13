# EMGExo — EMG-Controlled Exoskeleton

Real-time EMG-to-motor control system using a Jetson Orin Nano (brain) and ESP32 (spine) driving 3x AK80-9 actuators over CAN bus.

**Target latency:** < 10ms end-to-end (EMG signal to motor actuation).

## Architecture

```
MyoWare Sensors ──[analog]──> MCP3008 ADC ──[SPI1]──> Jetson Orin Nano
                                                            │
                                                       [Python]
                                                       EMG processing
                                                       Control mapping
                                                            │
                                                         [SPI0]
                                                            │
                                                         ESP32
                                                            │
                                                        [CAN Bus]
                                                       ┌────┼────┐
                                                   AK80-9  AK80-9  AK80-9
                                                   (ID 1)  (ID 2)  (ID 3)
```

## Directory Structure

```
EMGExo/
├── jetson/                          # Python — runs on Jetson Orin Nano
│   ├── emg/                         # EMG acquisition & processing (testable alone)
│   │   ├── emg_sensor.py            #   MCP3008 ADC reader
│   │   ├── emg_processor.py         #   EMA envelope detection
│   │   └── test_emg.py              #   Standalone EMG test
│   ├── motor/                       # Motor control via SPI (testable alone)
│   │   ├── motor_controller.py      #   SPI master + protocol
│   │   └── test_motors.py           #   Standalone motor test
│   ├── integration/                 # Full pipeline
│   │   ├── emg_exo_controller.py    #   EMG → motor control loop
│   │   └── test_integration.py      #   Full system test
│   └── requirements.txt
├── esp32/                           # C++ firmware — runs on ESP32
│   ├── platformio.ini
│   └── src/
│       └── main.cpp                 # Dual-mode: SPI slave or serial test
└── README.md
```

## Hardware Requirements

| Component | Purpose | Approx. Cost |
|---|---|---|
| Jetson Orin Nano | Main controller (Python) | ~$200 |
| ESP32 DevKit | CAN bus bridge | ~$10 |
| 3x AK80-9 motors | Actuators | — |
| CAN transceiver (SN65HVD230 or MCP2551) | CAN bus interface | ~$5 |
| MCP3008 | 10-bit SPI ADC for EMG | ~$3 |
| MyoWare 2.0 sensors (x3) | EMG acquisition | ~$40 each |

## Wiring

### Jetson to ESP32 (SPI0)

| Jetson Pin | Signal | ESP32 Pin |
|---|---|---|
| Pin 19 | SPI0_MOSI | GPIO 23 |
| Pin 21 | SPI0_MISO | GPIO 19 |
| Pin 23 | SPI0_SCLK | GPIO 18 |
| Pin 24 | SPI0_CS0 | GPIO 5 |
| Pin 6 | GND | GND |

### Jetson to MCP3008 (SPI1)

| Jetson Pin | Signal | MCP3008 Pin |
|---|---|---|
| Pin 37 | SPI1_MOSI | DIN |
| Pin 22 | SPI1_MISO | DOUT |
| Pin 13 | SPI1_SCLK | CLK |
| Pin 18 | SPI1_CS0 | CS/SHDN |
| 3.3V | VDD | VDD, VREF |
| GND | GND | AGND, DGND |

MyoWare SIG outputs connect to MCP3008 channels 0, 1, 2.

### ESP32 to CAN Transceiver

| ESP32 Pin | CAN Transceiver |
|---|---|
| GPIO 16 (SPI mode) or GPIO 5 (serial mode) | TX |
| GPIO 4 | RX |

CAN bus runs at 1 Mbps. All 3 motors are daisy-chained on the same bus (IDs 1, 2, 3).

## Quick Start

### 1. Flash ESP32

```bash
cd esp32

# For standalone testing (serial mode, default):
pio run -t upload

# For Jetson integration (SPI mode):
# Uncomment "#define USE_SPI_MODE" in src/main.cpp first
pio run -t upload
```

### 2. Test ESP32 + Motors (no Jetson needed)

With the ESP32 in serial test mode, open a serial monitor at 115200 baud:

```
INIT                          # Enter control mode
CMD,1,0.5,0,100,2,0          # Motor 1 to 0.5 rad
CMD,2,-0.3,0,100,2,0         # Motor 2 to -0.3 rad
CMD,3,0,0,100,2,0            # Motor 3 to zero
STOP                          # Exit control mode
```

### 3. Test EMG on Jetson (no motors needed)

```bash
cd jetson
pip install -r requirements.txt

# Read EMG from 3 channels for 5 seconds
python -m emg.test_emg --channels 0 1 2 --duration 5
```

### 4. Test Motors from Jetson (no EMG needed)

Flash ESP32 with `USE_SPI_MODE` enabled, then:

```bash
cd jetson

# Send sinusoidal trajectory to all 3 motors
python -m motor.test_motors --amplitude 0.3 --frequency 0.5 --duration 5
```

### 5. Full Integration

```bash
cd jetson
python -m integration.test_integration --duration 10 --rate 500
```

## Latency Budget

| Stage | Estimated |
|---|---|
| MyoWare analog output | ~0 ms |
| MCP3008 ADC read (SPI) | ~0.01 ms |
| EMG envelope (EMA filter) | ~1-3 ms |
| Control mapping | ~0.1 ms |
| Jetson SPI to ESP32 | ~0.07 ms |
| ESP32 CAN to 3 motors | ~0.5 ms |
| CAN feedback collection | ~0.5 ms |
| SPI response to Jetson | ~0.05 ms |
| **Total** | **~3-6 ms** |

## Motor Parameters (AK80-9)

| Parameter | Min | Max | Bits |
|---|---|---|---|
| Position | -12.5 rad | 12.5 rad | 16 |
| Velocity | -50.0 rad/s | 50.0 rad/s | 12 |
| Torque | -18.0 Nm | 18.0 Nm | 12 |
| Kp | 0 | 500 | 12 |
| Kd | 0 | 5 | 12 |

## Enabling SPI on Jetson Orin Nano

SPI must be enabled via the Jetson pinmux tool:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# Select "Configure Jetson 40pin Header"
# Enable SPI0 and SPI1
# Save and reboot
```

Verify SPI devices exist:
```bash
ls /dev/spidev*
# Should show /dev/spidev0.0 and /dev/spidev1.0
```
