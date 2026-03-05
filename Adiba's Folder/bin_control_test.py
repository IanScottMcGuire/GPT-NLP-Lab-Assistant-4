#!/usr/bin/env python3
"""
bin_controller.py - Jetson Orin Nano <-> ESP32 Bin Dispenser Controller
------------------------------------------------------------------------
Communicates over UART (RX/TX) with the ESP32 running the bin/motor sketch.

Hardware wiring:
  Jetson Pin 3 (ttyTHS1 TX) -> ESP32 RX0
  Jetson Pin 5 (ttyTHS1 RX) <- ESP32 TX0
  Jetson Pin 6 (GND)        -> ESP32 GND
  Motor powered by external PSU.

Usage:
  python3 bin_controller.py [--port /dev/ttyTHS1] [--baud 115200]

Supported commands:
  h        - Home the carousel (required on first boot)
  bin0..3  - Move carousel to bin 0, 1, 2, or 3
  i        - Run inventory sensing (after bin is reinserted + 2s lockout clears)
  e        - E-stop (disables motor immediately)
  status   - Print current connection/state info (local only)
  quit     - Exit this script
"""

import serial
import threading
import argparse
import sys
import time

# ── Default serial port for Jetson Orin Nano UART1 ──────────────────────────
DEFAULT_PORT = "/dev/ttyTHS2"
DEFAULT_BAUD = 115200

VALID_COMMANDS = {"h", "i", "e", "bin0", "bin1", "bin2", "bin3", "status", "quit", "help"}

# ── State tracked from ESP32 responses ──────────────────────────────────────
state = {
    "homed": False,
    "current_bin": None,
    "gate_blocked": False,
    "inventory_pending": False,
    "last_inventory_result": None,
    "last_inventory_distance_cm": None,
}


def parse_esp32_line(raw: str):
    line = raw.strip()
    if not line:
        return

    if "Homing complete" in line:
        state["homed"] = True
        state["current_bin"] = 0

    if "Now at BIN" in line:
        try:
            state["current_bin"] = int(line.split("BIN")[-1].strip())
        except ValueError:
            pass

    if "GATE: BLOCKED" in line:
        state["gate_blocked"] = True
    if "GATE: OPEN" in line or "GATE: Ready" in line:
        state["gate_blocked"] = False
    if "GATE: Ready" in line:
        state["inventory_pending"] = True

    if line == "HI":
        state["last_inventory_result"] = "HI (inventory present)"
        state["inventory_pending"] = False
    elif line == "LO":
        state["last_inventory_result"] = "LO (bin empty/low)"
        state["inventory_pending"] = False

    if line.startswith("(Distance(cm)"):
        try:
            val = line.strip("()").split("=")[-1].strip()
            state["last_inventory_distance_cm"] = val
        except Exception:
            pass

    if "Press 'i' to perform inventory" in line:
        state["inventory_pending"] = True


def reader_thread(ser: serial.Serial, stop_event: threading.Event):
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
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()


def main():
    parser = argparse.ArgumentParser(description="Jetson->ESP32 Bin Controller")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    print(f"Opening serial port {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        ser.dtr = False   # Prevent ESP32 from resetting when port opens
        ser.rts = False   # Prevent ESP32 from resetting when port opens
    except serial.SerialException as e:
        print(f"[FATAL] Could not open port: {e}")
        print("Try: sudo chmod 666 /dev/ttyTHS1")
        sys.exit(1)

    print(f"Connected. Waiting for ESP32 boot messages...\n")
    time.sleep(3)  # Give ESP32 time to boot and send prompt

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
                    print("[WARN] Gate appears blocked. Home anyway? (y/n): ", end="")
                    confirm = input().strip().lower()
                    if confirm != "y":
                        continue
                send_command(ser, "h")
                continue

            if user_input == "i":
                if not state["inventory_pending"]:
                    print("[LOCAL] Inventory not pending (no bin reinserted yet or already measured).")
                elif state["gate_blocked"]:
                    print("[LOCAL] Gate is BLOCKED. Push bin fully in first.")
                else:
                    send_command(ser, "i")
                continue

            if user_input in ("bin0", "bin1", "bin2", "bin3"):
                if state["gate_blocked"]:
                    print("[LOCAL] GATE BLOCKED. Push bin back in and wait for re-arm before selecting.")
                elif state["inventory_pending"]:
                    print("[LOCAL] INVENTORY REQUIRED. Press 'i' to measure before selecting another bin.")
                elif not state["homed"]:
                    print("[LOCAL] Not homed. Send 'h' first.")
                else:
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
