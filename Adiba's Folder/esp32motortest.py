#!/usr/bin/env python3
"""
Simple ESP32 echo test over UART TX/RX
Type anything -> sends to ESP32 -> prints whatever ESP32 sends back
"""

import serial
import threading
import time
import sys

SERIAL_PORT = '/dev/ttyTHS2'
BAUD_RATE   = 115200

def listen(ser, stop):
    while not stop.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"\n[ESP32] {line}")
                    print("SEND> ", end="", flush=True)
        except Exception:
            pass
        time.sleep(0.01)

def main():
    print(f"Opening {SERIAL_PORT} @ {BAUD_RATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.dtr = False
        ser.rts = False
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Run: sudo chmod 666 /dev/ttyTHS1")
        sys.exit(1)

    print("Connected. Listening for ESP32 output...")
    print("Type anything and press Enter to send. Ctrl+C to quit.\n")

    stop = threading.Event()
    t = threading.Thread(target=listen, args=(ser, stop), daemon=True)
    t.start()

    try:
        while True:
            print("SEND> ", end="", flush=True)
            msg = input().strip()
            if msg:
                ser.write((msg + "\n").encode("utf-8"))
                ser.flush()
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        stop.set()
        ser.close()

if __name__ == "__main__":
    main()
