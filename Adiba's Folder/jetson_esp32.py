#!/usr/bin/env python3
"""
Jetson Orin Nano Serial Communication with ESP32

TEMP TESTING: Listens for "HIGH" messages from ESP32 when button is pressed
Allows sending ON/OFF commands to control the ESP32's LED
pip install pyserial
jetson_esp32.py
"""
 
import serial
import time
import threading
import sys

# Serial port configuration
SERIAL_PORT = '/dev/ttyTHS2'  # Found port by using ls /dev/tty* and plugging/unplugging esp32 to see what port appeared and disappered
BAUD_RATE = 115200
TIMEOUT = 1  # seconds

class ESP32Communication:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
    def connect(self):
        """Establish serial connection with ESP32"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=TIMEOUT
            )
            time.sleep(2)  # Wait for connection to stabilize
            print(f"Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")
            return False
    
    def listen_for_button(self):
        """Listen for button press signal from ESP32"""
        print("\nListening for button presses from ESP32...")
        print("(Press Ctrl+C to stop)\n")
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    message = self.serial_conn.readline().decode('utf-8').strip()
                    if message:
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"{message}")
                        
                        # React to button press
                        if message == "HIGH":
                            print("Button was pressed.")
                            
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break
            except UnicodeDecodeError:
                pass  # Ignore decode errors
            except KeyboardInterrupt:
                break
                
            time.sleep(0.01)  # Small delay to prevent CPU spinning
    
    def send_command(self, command):
        """Send command to ESP32"""
        if self.serial_conn:
            try:
                self.serial_conn.write(f"{command}\n".encode('utf-8'))
                print(f"Sent to ESP32: {command}")
                time.sleep(0.1)  # Give ESP32 time to respond
            except serial.SerialException as e:
                print(f"Error sending command: {e}")
        else:
            print("Not connected to ESP32")
    
    def interactive_mode(self):
        print("Begin interactive mode. Press ON/OFF to toggle LED and q to quit. Press button to receive message from esp32.")
        
        # Start listener thread
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
                else:
                    print("Invalid command. Use ON, OFF, or q")
                    
            except KeyboardInterrupt:
                print("\nExiting...")
                break
        
        self.running = False
        time.sleep(0.5)  # Give listener thread time to finish
    
    def close(self):
        """Close serial connection"""
        if self.serial_conn:
            self.serial_conn.close()
            print("Connection closed")

def find_serial_ports():
    """Helper function to list available serial ports"""
    import glob
    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    return ports

if __name__ == "__main__":
    print("ESP32 <-> Jetson Communication Test")
    print("=" * 50)
    
    # Show available ports
    available_ports = find_serial_ports()
    if available_ports:
        print(f"Available serial ports: {', '.join(available_ports)}")
    else:
        print("No serial ports found!")
        print("Make sure ESP32 is connected via USB")
        sys.exit(1)
    
    # Create communication object
    esp32 = ESP32Communication(SERIAL_PORT, BAUD_RATE)
    
    # Connect to ESP32
    if esp32.connect():
        try:
            # Run interactive mode
            esp32.interactive_mode()
        finally:
            esp32.close()
    else:
        print("\nFailed to connect.")
