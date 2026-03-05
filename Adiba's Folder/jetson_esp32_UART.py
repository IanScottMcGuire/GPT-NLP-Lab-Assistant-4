#!/usr/bin/env python3
"""
Jetson Orin Nano Serial Communication with ESP32
Using direct UART TX/RX GPIO connection (NOT USB)

Wiring:
  Jetson Pin 1  (3.3V)  -> ESP32 3V3
  Jetson GND            -> ESP32 GND
  Jetson TX             -> ESP32 RX
  Jetson RX             -> ESP32 TX

Serial port: /dev/ttyTHS2 (Jetson hardware UART on GPIO header)

pip install pyserial
Usage: python3 jetson_esp32.py
"""

import serial
import time
import threading
import sys

# --- Serial port configuration ---
# /dev/ttyTHS2 is the Jetson's hardware UART on the GPIO header (TX/RX pins)
# This is correct for direct wired TX/RX — do NOT use /dev/ttyUSB* here
SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 115200
TIMEOUT = 1  # seconds


class ESP32Communication:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False

    def connect(self):
        """Establish UART serial connection with ESP32"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT,
                # Disable hardware/software flow control — not used in simple TX/RX wiring
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            # NOTE: No time.sleep(2) reset wait needed here.
            # Unlike USB, opening a direct UART connection does NOT
            # trigger an ESP32 reset, so no boot delay is necessary.
            self.serial_conn.reset_input_buffer()  # Clear any stale bytes
            print(f"Connected to ESP32 via UART on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            print("\nTroubleshooting:")
            print("  1. Check TX/RX are not swapped (Jetson TX -> ESP32 RX, Jetson RX -> ESP32 TX)")
            print("  2. Confirm /dev/ttyTHS2 is enabled: sudo systemctl status nvgetty")
            print("  3. Disable getty on this port if active:")
            print("       sudo systemctl stop nvgetty")
            print("       sudo systemctl disable nvgetty")
            print("  4. Check permissions: sudo chmod 666 /dev/ttyTHS2")
            return False

    def listen_for_button(self):
        """Listen for button press signal from ESP32 (runs in background thread)"""
        print("\nListening for button presses from ESP32...")
        print("(Press Ctrl+C to stop)\n")

        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    message = self.serial_conn.readline().decode('utf-8').strip()
                    if message:
                        timestamp = time.strftime("%H:%M:%S")

                        if message == "HIGH":
                            print(f"\n[{timestamp}] *** Button PRESSED on ESP32 ***")
                        elif message == "LED turned ON":
                            print(f"\n[{timestamp}] ESP32 confirmed: LED ON")
                        elif message == "LED turned OFF":
                            print(f"\n[{timestamp}] ESP32 confirmed: LED OFF")
                        else:
                            print(f"\n[{timestamp}] ESP32: {message}")

                        print("Enter command (ON/OFF/q): ", end="", flush=True)

            except serial.SerialException as e:
                print(f"\nSerial error: {e}")
                break
            except UnicodeDecodeError:
                pass  # Ignore occasional noise on the line
            except KeyboardInterrupt:
                break

            time.sleep(0.01)

    def send_command(self, command):
        """Send a newline-terminated command to the ESP32"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(f"{command}\n".encode('utf-8'))
                print(f"Sent to ESP32: {command}")
                time.sleep(0.1)
            except serial.SerialException as e:
                print(f"Error sending command: {e}")
        else:
            print("Not connected to ESP32")

    def interactive_mode(self):
        print("Interactive mode ready.")
        print("  ON  -> Turn LED on")
        print("  OFF -> Turn LED off")
        print("  q   -> Quit")
        print("Button presses from ESP32 will appear automatically.\n")

        self.running = True
        listener_thread = threading.Thread(target=self.listen_for_button, daemon=True)
        listener_thread.start()

        while True:
            try:
                command = input("Enter command (ON/OFF/q): ").strip()

                if command.lower() == 'q':
                    print("Exiting...")
                    break
                elif command.upper() in ['ON', 'OFF']:
                    self.send_command(command.upper())
                elif command == '':
                    continue
                else:
                    print(f"Unknown command: '{command}'. Use ON, OFF, or q.")

            except KeyboardInterrupt:
                print("\nInterrupted. Exiting...")
                break

        self.running = False
        time.sleep(0.5)

    def close(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Connection closed.")


if __name__ == "__main__":
    print("ESP32 <-> Jetson UART Communication Test")
    print("=" * 50)
    print(f"Port: {SERIAL_PORT}  |  Baud: {BAUD_RATE}")
    print("Wiring: Jetson TX -> ESP32 RX  |  Jetson RX -> ESP32 TX")
    print("=" * 50)

    esp32 = ESP32Communication(SERIAL_PORT, BAUD_RATE)

    if esp32.connect():
        try:
            esp32.interactive_mode()
        finally:
            esp32.close()
    else:
        sys.exit(1)
