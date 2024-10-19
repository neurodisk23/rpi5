import serial
import time

# Replace '/dev/ttyACM0' with your actual port
port = '/dev/ttyACM0'
baudrate = 57600  # Set this to the baud rate of your CubePilot

ser = None  # Initialize ser to None

try:
    # Initialize serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {ser.name}")

    while True:
        if ser.in_waiting > 0:  # Check if there is data available
            try:
                line = ser.readline().decode('utf-8').rstrip()  # Attempt to decode as UTF-8
            except UnicodeDecodeError:
                # If decoding fails, read the raw bytes and decode using 'latin-1' or ignore errors
                line = ser.readline()  # Read raw bytes
                line = line.decode('latin-1', errors='ignore').rstrip()  # Decode with fallback
            print(line)  # Print the received data

except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    if ser is not None:  # Check if ser was successfully created before closing
        ser.close()  # Ensure the connection is closed when done
