#!/usr/bin/env python3
"""Reset ESP32 and capture debug output to see what firmware is running."""
import serial, time, threading

DBG = "/dev/ttyUSB0"   # FTDI debug
CMD = "/dev/ttyUSB2"   # CP2102 command

stop = False
def reader(s):
    while not stop:
        d = s.readline()
        if d:
            print(f"[DEBUG] {d.decode(errors='replace').rstrip()}")

# Open debug port first
dbg = serial.Serial(DBG, 115200, timeout=0.5)
dbg.reset_input_buffer()
t = threading.Thread(target=reader, args=(dbg,), daemon=True)
t.start()

# Open command port (triggers DTR reset)
print("Resetting ESP32...")
cmd = serial.Serial(CMD, 115200)
time.sleep(0.1)

# Listen for 12 seconds to capture full setup()
print("Listening for 12 seconds...\n")
time.sleep(12)

stop = True
cmd.close()
dbg.close()
print("\nDone.")
