import serial
import time

# Set up serial connection for GPS
gps_port = '/dev/serial0'  # Use /dev/serial0 on Raspberry Pi
gps_serial = serial.Serial(gps_port, baudrate=9600, timeout=1)

# Function to read GPS data
def read_gps():
    while True:
        if gps_serial.in_waiting > 0:
            data = gps_serial.readline().decode('ascii', errors='replace')
            if data.startswith('$GPGGA'):
                print(data.strip())  # Print to terminal
                time.sleep(1)  # Delay for readability

try:
    print("Starting GPS reader...")
    read_gps()
except KeyboardInterrupt:
    print("Program stopped.")
finally:
    gps_serial.close()
