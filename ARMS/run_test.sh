#!/usr/bin/env bash
# One-command motor test: log + trajectory + plot
# Uses EXACT same workflow as the "it worked!" run, just at 30% amplitude.
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

CMD_PORT="/dev/ttyUSB2"
DBG_PORT="/dev/ttyUSB0"
RAW_LOG="motor_raw.log"
CSV="motor_log.csv"

for p in "$CMD_PORT" "$DBG_PORT"; do
    if [ ! -e "$p" ]; then
        echo "ERROR: $p not found."
        exit 1
    fi
done

if [ ! -f "./serial_write_wrist_slow" ]; then
    echo "Compiling serial_write_wrist_slow.cpp..."
    g++ -o serial_write_wrist_slow serial_write_wrist_slow.cpp -lpthread -std=c++14
fi

# Capture debug serial with cat (lightweight, no Python serial overhead)
echo "Capturing debug feedback from $DBG_PORT..."
stty -F "$DBG_PORT" 115200 raw -echo
cat "$DBG_PORT" > "$RAW_LOG" &
CAT_PID=$!
sleep 0.3

# Run trajectory (includes 8s ESP32 boot wait)
echo "Running trajectory (30% amplitude)..."
./serial_write_wrist_slow

# Stop capture
kill $CAT_PID 2>/dev/null || true
wait $CAT_PID 2>/dev/null || true
# Kill any stray cat on the debug port
pkill -f "cat $DBG_PORT" 2>/dev/null || true
sleep 0.3

# Parse raw log into CSV (round-robin M1,M2,M3)
echo "Parsing feedback..."
python3 -c "
import csv, sys
raw = open('$RAW_LOG', 'r', errors='replace').read()
vals = [v.strip() for v in raw.split(';') if v.strip()]
with open('$CSV', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['sample', 'motor_id', 'position_rad'])
    mid = 0
    for i, v in enumerate(vals):
        try:
            pos = float(v)
        except ValueError:
            continue
        motor = (mid % 3) + 1
        mid += 1
        w.writerow([i, motor, f'{pos:.4f}'])
print(f'Parsed {mid} samples')
"

# Plot
if [ -f "$CSV" ]; then
    echo "Plotting..."
    python3 plot_motors.py "$CSV"
else
    echo "No data captured."
fi
