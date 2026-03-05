import serial
import time

# Adjust port if needed — check with: ls /dev/ttyTHS*  or  ls /dev/ttyS*
SERIAL_PORT = "/dev/ttyTHS2"
BAUD_RATE = 115200
TIMEOUT = 5  # seconds to wait for a response

def main():
    print(f"Opening serial port {SERIAL_PORT} at {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
    except serial.SerialException as e:
        print(f"[ERROR] Could not open port: {e}")
        print("Try running with sudo or add your user to the 'dialout' group:")
        print("  sudo usermod -aG dialout $USER")
        return

    time.sleep(1)  # Let port settle

    message = "Hello from Jetson Orin Nano!"
    print(f"\n[Jetson] Sending: {message}")
    ser.write((message + "\n").encode("utf-8"))

    print("[Jetson] Waiting for response from ESP32...")
    response = ser.readline().decode("utf-8").strip()

    if response:
        print(f"[Jetson] Received from ESP32: {response}")
    else:
        print("[Jetson] No response received within timeout. Check wiring and port.")

    ser.close()
    print("\nDone. Port closed.")

if __name__ == "__main__":
    main()
