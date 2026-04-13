#!/usr/bin/env bash
set -euo pipefail

# ── Colors ──────────────────────────────────────────────────────────────
R='\033[1;91m'  G='\033[1;92m'  Y='\033[1;93m'
B='\033[1;94m'  C='\033[1;96m'  W='\033[1;97m'
M='\033[1;95m'  D='\033[2m'     RST='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RT_DIR="$SCRIPT_DIR/jetson/rt"
BUILD_DIR="$RT_DIR/build"
BINARY="$BUILD_DIR/emgexo_rt"

banner() {
    echo -e "${C}"
    echo "  ╔══════════════════════════════════════════════════╗"
    echo "  ║          EMGExo RT — Launch Manager              ║"
    echo "  ╚══════════════════════════════════════════════════╝"
    echo -e "${RST}"
}

detect_uart() {
    for dev in /dev/ttyUSB* /dev/ttyTHS1 /dev/ttyACM*; do
        if [ -e "$dev" ] 2>/dev/null; then
            echo "$dev"
            return
        fi
    done
    echo ""
}

list_uarts() {
    local found=0
    for dev in /dev/ttyUSB* /dev/ttyTHS* /dev/ttyACM*; do
        if [ -e "$dev" ] 2>/dev/null; then
            echo -e "    ${G}$dev${RST}"
            found=1
        fi
    done
    if [ $found -eq 0 ]; then
        echo -e "    ${D}(none found)${RST}"
    fi
}

setup_uart() {
    echo ""
    # Collect all serial ports into an array
    local ports=()
    for dev in /dev/ttyUSB* /dev/ttyTHS* /dev/ttyACM*; do
        if [ -e "$dev" ] 2>/dev/null; then
            ports+=("$dev")
        fi
    done

    if [ ${#ports[@]} -eq 0 ]; then
        echo -e "${D}No serial ports found — running without motors${RST}"
        UART_ARG=""
        return
    fi

    echo -e "${W}Select ESP32 serial port:${RST}"
    local i=1
    for p in "${ports[@]}"; do
        echo -e "  ${G}${i}${RST}) ${G}${p}${RST}"
        ((i++))
    done
    echo -e "  ${D}${i}${RST}) ${D}Skip (no motor output)${RST}"
    echo ""
    read -rp "$(echo -e ${W}"Port [1-${i}]: "${RST})" PORT_CHOICE

    if [[ "$PORT_CHOICE" =~ ^[0-9]+$ ]] && [ "$PORT_CHOICE" -ge 1 ] && [ "$PORT_CHOICE" -le ${#ports[@]} ]; then
        local sel="${ports[$((PORT_CHOICE - 1))]}"
        echo -e "${G}Motor output → $sel${RST}"
        UART_ARG="--uart $sel"
    else
        echo -e "${D}Skipping motor output${RST}"
        UART_ARG=""
    fi
}

# ── Main ────────────────────────────────────────────────────────────────

banner

echo -e "${W}Select mode:${RST}"
echo ""
echo -e "  ${G}1${RST}) ${G}Training (Healthy)${RST}        CNN + PhysioMio healthy-arm data"
echo -e "  ${Y}2${RST}) ${Y}Rehabilitation (Stroke)${RST}   CNN + PhysioMio impaired-arm data"
echo -e "  ${D}──────────────────────────────────────────────────────────${RST}"
echo -e "  ${B}3${RST}) ${B}Simulation + Motor${RST}        keyboard gestures → real motor actuation"
echo -e "  ${R}4${RST}) ${R}Live (Jetson only)${RST}        real SPI + TensorRT + UART"
echo ""
read -rp "$(echo -e ${W}"Choice [1/2/3/4]: "${RST})" MODE_CHOICE

CMAKE_FLAGS=""
RUN_ARGS=""
UART_ARG=""

case "$MODE_CHOICE" in
    1)
        MODE_NAME="Training (Healthy)"
        CMAKE_FLAGS="-DLAPTOP_SIM=ON"
        DEFAULT_BIN="$SCRIPT_DIR/emg_bin_healthy"

        if [ -d "$DEFAULT_BIN" ] && ls "$DEFAULT_BIN"/*.bin &>/dev/null; then
            echo -e "${G}Found dataset: $DEFAULT_BIN${RST}"
            BIN_DIR="$DEFAULT_BIN"
        else
            echo -e "${R}Healthy dataset not found at $DEFAULT_BIN${RST}"
            echo -e "${D}Run the data pipeline:${RST}"
            echo -e "${D}  cd jetson${RST}"
            echo -e "${D}  python data_prep.py${RST}"
            echo -e "${D}  python train.py --arm healthy${RST}"
            echo -e "${D}  python tools/convert_npz_to_bin.py --npz prepared_data/healthy_train.npz --out ../emg_bin_healthy/${RST}"
            echo -e "${D}  python tools/precompute_predictions.py --model checkpoints/healthy_model.pth --npz prepared_data/healthy_train.npz --bin_dir ../emg_bin_healthy/${RST}"
            exit 1
        fi

        setup_uart
        RUN_ARGS="--dataset $BIN_DIR --mode Training --sim --no-reset $UART_ARG"
        ;;
    2)
        MODE_NAME="Rehabilitation (Stroke)"
        CMAKE_FLAGS="-DLAPTOP_SIM=ON"
        DEFAULT_BIN="$SCRIPT_DIR/emg_bin_stroke"

        if [ -d "$DEFAULT_BIN" ] && ls "$DEFAULT_BIN"/*.bin &>/dev/null; then
            echo -e "${G}Found dataset: $DEFAULT_BIN${RST}"
            BIN_DIR="$DEFAULT_BIN"
        else
            echo -e "${R}Stroke dataset not found at $DEFAULT_BIN${RST}"
            echo -e "${D}Run the data pipeline:${RST}"
            echo -e "${D}  cd jetson${RST}"
            echo -e "${D}  python data_prep.py${RST}"
            echo -e "${D}  python train.py --arm impaired${RST}"
            echo -e "${D}  python tools/convert_npz_to_bin.py --npz prepared_data/impaired_train.npz --out ../emg_bin_stroke/${RST}"
            echo -e "${D}  python tools/precompute_predictions.py --model checkpoints/stroke_model.pth --npz prepared_data/impaired_train.npz --bin_dir ../emg_bin_stroke/${RST}"
            exit 1
        fi

        setup_uart
        RUN_ARGS="--dataset $BIN_DIR --mode Rehab --sim --no-reset $UART_ARG"
        ;;
    3)
        MODE_NAME="Simulation + Motor"
        CMAKE_FLAGS="-DLAPTOP_SIM=ON"

        echo -e "${W}Available serial ports:${RST}"
        list_uarts
        echo ""

        DETECTED_UART="$(detect_uart)"
        if [ -n "$DETECTED_UART" ]; then
            read -rp "$(echo -e ${C}"UART device [$DETECTED_UART]: "${RST})" UART_DEV
            UART_DEV="${UART_DEV:-$DETECTED_UART}"
        else
            echo -e "${Y}No UART device auto-detected. Is the ESP32 plugged in?${RST}"
            read -rp "$(echo -e ${C}"UART device path: "${RST})" UART_DEV
        fi

        if [ ! -e "$UART_DEV" ]; then
            echo -e "${R}Error: $UART_DEV does not exist.${RST}"
            exit 1
        fi

        echo -e "${D}Waiting 3s for ESP32 to finish setup()...${RST}"
        sleep 3

        RUN_ARGS="--sim --no-reset --uart $UART_DEV"
        ;;
    4)
        MODE_NAME="Live (Jetson)"
        CMAKE_FLAGS="-DSIM=OFF"

        if [ "$EUID" -ne 0 ]; then
            echo ""
            echo -e "${Y}Live mode requires root for UART and RT scheduling.${RST}"
            read -rp "$(echo -e ${W}"Re-launch with sudo? [Y/n]: "${RST})" SUDO_ANS
            SUDO_ANS="${SUDO_ANS:-Y}"
            if [[ "$SUDO_ANS" =~ ^[Yy] ]]; then
                echo -e "${D}Re-launching with sudo...${RST}"
                exec sudo "$0" "$@"
            else
                echo -e "${R}Continuing without sudo — UART/RT may fail.${RST}"
            fi
        fi

        read -rp "$(echo -e ${C}"TensorRT engine path: "${RST})" ENGINE_PATH
        if [ ! -f "$ENGINE_PATH" ]; then
            echo -e "${R}Error: Engine file not found: $ENGINE_PATH${RST}"
            exit 1
        fi

        DETECTED_UART="$(detect_uart)"
        if [ -n "$DETECTED_UART" ]; then
            echo -e "${G}Detected UART: $DETECTED_UART${RST}"
            read -rp "$(echo -e ${C}"UART device [$DETECTED_UART]: "${RST})" UART_DEV
            UART_DEV="${UART_DEV:-$DETECTED_UART}"
        else
            echo -e "${Y}No UART device auto-detected.${RST}"
            read -rp "$(echo -e ${C}"UART device path: "${RST})" UART_DEV
        fi

        if [ ! -e "$UART_DEV" ]; then
            echo -e "${R}Warning: $UART_DEV does not exist.${RST}"
        fi

        RUN_ARGS="--engine $ENGINE_PATH --uart $UART_DEV"
        ;;
    *)
        echo -e "${R}Invalid choice.${RST}"
        exit 1
        ;;
esac

echo ""
echo -e "${W}Mode:${RST}  ${G}$MODE_NAME${RST}"
echo -e "${W}Args:${RST}  ${D}$RUN_ARGS${RST}"
echo ""

# ── Build ───────────────────────────────────────────────────────────────

echo -e "${C}Building...${RST}"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. $CMAKE_FLAGS 2>&1 | tail -1
make -j"$(nproc)" 2>&1

if [ ! -x "$BINARY" ]; then
    echo -e "${R}Build failed — binary not found.${RST}"
    exit 1
fi

echo ""
echo -e "${G}Build successful.${RST}"
echo ""

# ── Run ─────────────────────────────────────────────────────────────────

echo -e "${W}Launching:${RST} ${D}$BINARY $RUN_ARGS${RST}"
echo -e "${D}Press Ctrl+C to stop.${RST}"
echo ""

# shellcheck disable=SC2086
export EMGEXO_ROOT="$SCRIPT_DIR"
exec "$BINARY" $RUN_ARGS
