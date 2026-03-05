#!/usr/bin/env python3
"""
bin_controller.py - Jetson Orin Nano <-> ESP32 Bin Dispenser Controller
------------------------------------------------------------------------
Communicates over direct UART TX/RX GPIO connection with the ESP32.

Hardware wiring:
  Jetson TX (ttyTHS1) -> ESP32 GPIO 11 (RX)
  Jetson RX (ttyTHS1) -> ESP32 GPIO 9  (TX)
  Jetson GND          -> ESP32 GND
  Jetson 3.3V (Pin 1) -> ESP32 3V3

CSV logging:
  Every HI/LO inventory result is saved to inventory_log.csv in the
  same directory as this script, with a timestamp and distance reading.

Usage:
  python3 bin_controller.py [--port /dev/ttyTHS1] [--baud 115200]
"""

import serial
import threading
import argparse
import sys
import time
import os
import csv
from datetime import datetime

# ── Default serial port ──────────────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 115200

# ── CSV log file (saved next to this script) ─────────────────────────────────
LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "inventory_log.csv")
CSV_HEADERS = ["timestamp", "result", "distance_cm", "bin"]

# ── State tracked from ESP32 responses ──────────────────────────────────────
state = {
    "homed": False,
    "current_bin": None,
    "gate_blocked": False,
    "inventory_pending": False,
    "last_inventory_result": None,
    "last_inventory_distance_cm": None,
    # Pending values — held until we have both result AND distance
    "_pending_result": None,
    "_pending_distance": None,
}


def init_csv():
    """Create the CSV file with headers if it doesn't already exist."""
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADERS)
        print(f"[LOG] Created inventory log: {LOG_FILE}")
    else:
        print(f"[LOG] Appending to existing log: {LOG_FILE}")


def log_inventory(result: str, distance_cm: str, bin_num):
    """Append one inventory reading to the CSV file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    row = [timestamp, result, distance_cm if distance_cm else "N/A", bin_num if bin_num is not None else "N/A"]
    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)
    print(f"[LOG] Saved → {row}")


def parse_esp32_line(raw: str):
    """Parse incoming ESP32 output, update state, and log inventory results."""
    line = raw.strip()
    if not line:
        return

    # --- Homing ---
    if "Homing complete" in line:
        state["homed"] = True
        state["current_bin"] = 0

    # --- Current bin after move ---
    if "Now at BIN" in line:
        try:
            state["current_bin"] = int(line.split("BIN")[-1].strip())
        except ValueError:
            pass

    # --- Gate state ---
    if "GATE: BLOCKED" in line:
        state["gate_blocked"] = True
    if "GATE: OPEN" in line or "GATE: Ready" in line:
        state["gate_blocked"] = False
    if "GATE: Ready" in line:
        state["inventory_pending"] = True

    # --- Inventory result (HI or LO) ---
    # Hold the result until we also receive the distance on the next line
    if line == "HI":
        state["last_inventory_result"] = "HI"
        state["inventory_pending"] = False
        state["_pending_result"] = "HI"

    elif line == "LO":
        state["last_inventory_result"] = "LO"
        state["inventory_pending"] = False
        state["_pending_result"] = "LO"

    # --- Distance reading — arrives right after HI/LO ---
    # Format from ESP32: "(Distance(cm) = 8.243)"
    elif line.startswith("(Distance(cm)"):
        try:
            distance = line.strip("()").split("=")[-1].strip()
            state["last_inventory_distance_cm"] = distance
            state["_pending_distance"] = distance
        except Exception:
            state["_pending_distance"] = "N/A"

        # We now have both result and distance — write to CSV
        if state["_pending_result"] is not None:
            log_inventory(
                result=state["_pending_result"],
                distance_cm=state["_pending_distance"],
                bin_num=state["current_bin"]
            )
            state["_pending_result"] = None
            state["_pending_distance"] = None

    # --- NAN distance (no object detected) ---
    elif line == "(Distance(cm) = NAN)":
        state["last_inventory_distance_cm"] = "NAN"
        if state["_pending_result"] is not None:
            log_inventory(
                result=state["_pending_result"],
                distance_cm="NAN",
                bin_num=state["current_bin"]
            )
            state["_pending_result"] = None
            state["_pending_distance"] = None

    # --- Inventory pending prompt ---
    if "Press 'i' to perform inventory" in line:
        state["inventory_pending"] = True


def reader_thread(ser: serial.Serial, stop_event: threading.Event):
    """Background thread: read lines from ESP32 and print them."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                raw = ser.readline().decode("utf-8", errors="replace")
                if raw.strip():
                    parse_esp32_line(raw)
                    print(f"\r[ESP32] {raw.strip()}")
                    print("CMD> ", end="", flush=True)
        except serial.SerialException:
            print("\n[ERROR] Serial connection lost.")
            stop_event.set()
            break
        except Exception as e:
            print(f"\n[ERROR] Reader: {e}")
        time.sleep(0.01)


def print_status():
    print(f"  Homed          : {state['homed']}")
    print(f"  Current bin    : {state['current_bin']}")
    print(f"  Gate blocked   : {state['gate_blocked']}")
    print(f"  Inventory pend : {state['inventory_pending']}")
    print(f"  Last inv result: {state['last_inventory_result']}")
    print(f"  Last distance  : {state['last_inventory_distance_cm']} cm")
    print(f"  Log file       : {LOG_FILE}")


def print_help():
    print("""
Commands:
  h          - Home carousel (find BIN0)
  bin0..bin3 - Move to bin 0, 1, 2, or 3
  i          - Run inventory sensing (after gate re-arms)
  e          - E-stop motor immediately
  status     - Show local state summary
  help       - This help message
  quit       - Exit script
""")


def send_command(ser: serial.Serial, cmd: str):
    """Send a newline-terminated command string to ESP32."""
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()


def main():
    parser = argparse.ArgumentParser(description="Jetson->ESP32 Bin Controller")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    # Initialize CSV log file
    init_csv()

    # Force correct baud rate on Jetson hardware UART
    print(f"Configuring {args.port} at {args.baud} baud (raw mode)...")
    ret = os.system(f"sudo stty -F {args.port} raw {args.baud} cs8 -cstopb -parenb")
    if ret != 0:
        print(f"[WARN] stty command failed (exit {ret}). Continuing anyway...")

    print(f"Opening serial port {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"[FATAL] Could not open port: {e}")
        print(f"Try:  sudo chmod 666 {args.port}")
        print(f"  or: sudo systemctl stop nvgetty && sudo systemctl disable nvgetty")
        sys.exit(1)

    print(f"Connected. Waiting for ESP32 boot messages...\n")
    time.sleep(1.0)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    print_help()
    print("CMD> ", end="", flush=True)

    try:
        while not stop_event.is_set():
            try:
                user_input = input("CMD> ").strip().lower()
            except EOFError:
                break

            if not user_input:
                continue

            if user_input == "quit":
                for i in range(20):
                  send_command(ser,"quit")
                print("Exiting.")
                break

            if user_input == "help":
                print_help()
                continue

            if user_input == "status":
                print_status()
                continue

            if user_input == "e":
                print("[LOCAL] Sending E-STOP...")
                send_command(ser, "e")
                continue

            if user_input == "h":
                if state["gate_blocked"]:
                    confirm = input("[WARN] Gate appears blocked. Home anyway? (y/n): ").strip().lower()
                    if confirm != "y":
                        continue
                for i in range(20):
                  send_command(ser,"h")
                continue

            if user_input == "i":
                if not state["inventory_pending"]:
                    print("[LOCAL] Inventory not pending (no bin reinserted yet or already measured).")
                elif state["gate_blocked"]:
                    print("[LOCAL] Gate is BLOCKED. Push bin fully in first.")
                else:
                    for i in range(20):
                    	send_command(ser, "i")
                continue

            if user_input in ("bin0", "bin1", "bin2", "bin3"):
                if state["gate_blocked"]:
                    print("[LOCAL] GATE BLOCKED. Push bin back in and wait for re-arm.")
                elif state["inventory_pending"]:
                    print("[LOCAL] INVENTORY REQUIRED. Press 'i' before selecting another bin.")
                elif not state["homed"]:
                    print("[LOCAL] Not homed. Send 'h' first.")
                else:
                    for i in range(20):
                    	send_command(ser, user_input)
                continue

            print(f"[LOCAL] Unknown command '{user_input}'. Type 'help' for options.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt. Sending E-stop...")
        try:
            send_command(ser, "e")
        except Exception:
            pass

    finally:
        stop_event.set()
        t.join(timeout=1)
        ser.close()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
