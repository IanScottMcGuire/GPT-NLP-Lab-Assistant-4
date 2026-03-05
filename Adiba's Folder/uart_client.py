#!/usr/bin/env python3
"""
UART Client for Component Dispenser
Sends bin numbers to ESP32 and waits for confirmation
"""

import serial
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Update this to your ESP32's port
BAUD_RATE = 115200
TIMEOUT = 5  # seconds to wait for response

# Component to bin mapping
COMPONENT_TO_BIN = {
    "1kohm": 0,
    "1k": 0,
    "1kΩ": 0,
    "10kohm": 1,
    "10k": 1,
    "10kΩ": 1,
    "100ohm": 2,
    "100": 2,
    "100Ω": 2,
    "100kohm": 3,
    "100k": 3,
    "100kΩ": 3
}

class ESP32Client:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        
    def connect(self):
        """Establish serial connection with ESP32"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=TIMEOUT
            )
            time.sleep(2)  # Wait for connection to stabilize
            print(f"[UART] Connected to ESP32 on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[UART ERROR] Failed to connect to {self.port}: {e}")
            return False
    
    def send_bin_number(self, bin_number):
        """
        Send bin number to ESP32 and wait for HIGH response
        
        Args:
            bin_number (int): Bin number (0-3)
            
        Returns:
            bool: True if HIGH received, False otherwise
        """
        if not self.serial_conn:
            print("[UART ERROR] Not connected to ESP32")
            return False
        
        if not isinstance(bin_number, int) or bin_number < 0 or bin_number > 3:
            print(f"[UART ERROR] Invalid bin number: {bin_number}. Must be 0-3")
            return False
        
        try:
            # Send bin number
            command = f"{bin_number}\n"
            self.serial_conn.write(command.encode('utf-8'))
            print(f"[UART] Sent bin number: {bin_number}")
            
            # Wait for HIGH response
            start_time = time.time()
            while time.time() - start_time < TIMEOUT:
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode('utf-8').strip()
                    print(f"[UART] ESP32 response: {response}")
                    
                    if response == "HIGH":
                        print("[UART] ✓ Dispense confirmed!")
                        return True
                    elif "BIN" in response or "DISPENSING" in response:
                        # ESP32 acknowledging the command
                        continue
                    
                time.sleep(0.1)
            
            print("[UART] ✗ Timeout waiting for HIGH response")
            return False
            
        except serial.SerialException as e:
            print(f"[UART ERROR] Communication error: {e}")
            return False
    
    def close(self):
        """Close serial connection"""
        if self.serial_conn:
            self.serial_conn.close()
            print("[UART] Connection closed")

# Global client instance
_esp32_client = None

def get_esp32_client():
    """Get or create ESP32 client singleton"""
    global _esp32_client
    if _esp32_client is None:
        _esp32_client = ESP32Client()
        _esp32_client.connect()
    return _esp32_client

def send_component_request(component_name):
    """
    Send component request to ESP32
    
    Args:
        component_name (str): Component name (e.g., "1kohm", "10k", "100ohm")
        
    Returns:
        bool: True if dispense confirmed, False otherwise
    """
    # Normalize component name
    component_name = component_name.lower().replace(" ", "").replace("ω", "")
    
    # Map to bin number
    bin_number = COMPONENT_TO_BIN.get(component_name)
    
    if bin_number is None:
        print(f"[UART ERROR] Unknown component: {component_name}")
        print(f"[UART] Available components: {list(COMPONENT_TO_BIN.keys())}")
        return False
    
    print(f"[UART] Component '{component_name}' -> Bin {bin_number}")
    
    # Get client and send
    client = get_esp32_client()
    return client.send_bin_number(bin_number)

def close_connection():
    """Close ESP32 connection"""
    global _esp32_client
    if _esp32_client:
        _esp32_client.close()
        _esp32_client = None

if __name__ == "__main__":
    # Test script
    print("ESP32 UART Client Test")
    print("=" * 50)
    
    client = ESP32Client()
    if client.connect():
        print("\nTesting bin requests...")
        
        # Test each bin
        for bin_num in range(4):
            print(f"\n→ Testing Bin {bin_num}")
            success = client.send_bin_number(bin_num)
            print(f"  Result: {'SUCCESS' if success else 'FAILED'}")
            time.sleep(2)
        
        client.close()
    else:
        print("Failed to connect to ESP32")
