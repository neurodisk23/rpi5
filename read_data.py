import serial
import time

# Replace '/dev/ttyACM0' with your actual port
port = '/dev/ttyACM0'
baudrate = 9600  # Set this to the baud rate of your CubePilot

ser = None  # Initialize ser to None

try:
    # Initialize serial connection
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {ser.name}")

    while True:
        if ser.in_waiting > 0:  # Check if there is data available
            line = ser.readline().decode('utf-8').rstrip()  # Read a line and decode it
            print(line)  # Print the received data

except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    if ser is not None:  # Check if ser was successfully created before closing
        ser.close()  # Ensure the connection is closed when done
