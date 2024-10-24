import serial
import time

# Configure the serial connection
serial_port = '/dev/ttyAMA0'  # Adjust this to your port
baud_rate = 9600                # Common baud rate for GPS modules

# Function to parse GPS data
def parse_nmea_sentence(sentence):
    parts = sentence.split(',')
    speed = None
    num_satellites = None
    hdop = None

    # Parse $GPRMC for speed
    if parts[0] == '$GPRMC' and parts[2] == 'A':
        speed = float(parts[7]) * 1.15078  # Convert knots to mph (optional)

    # Parse $GPGSV for number of satellites
    elif parts[0] == '$GPGSV':
        num_satellites = int(parts[3])  # Total number of satellites in view

    # Parse $GNGNS for HDOP (if available)
    elif parts[0] == '$GNGNS':
        hdop = float(parts[8])  # HDOP value

    return speed, num_satellites, hdop

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    print("Starting GPS data read...")
    
    speed = None
    num_satellites = None
    hdop = None

    try:
        while True:
            # Read a line from the GPS module
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                # Parse the line
                speed, num_satellites, hdop = parse_nmea_sentence(line)

                if speed is not None:
                    print(f"Speed: {speed:.2f} mph")
                
                if num_satellites is not None:
                    print(f"Number of Satellites: {num_satellites}")
                
                if hdop is not None:
                    print(f"HDOP: {hdop:.2f}")

    except KeyboardInterrupt:
        print("Exiting...")
