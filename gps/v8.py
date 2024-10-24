import serial
import time

# Configure the serial connection
serial_port = '/dev/ttyAMA0'  # Adjust this to your port
baud_rate = 9600                # Common baud rate for GPS modules

# Function to parse GPS data
def parse_nmea_sentence(sentence):
    parts = sentence.split(',')
    speed = None
    latitude = None
    longitude = None
    num_satellites = None
    hdop = None

    # Parse $GPRMC for speed, latitude, and longitude
    if parts[0] == '$GPRMC' and parts[2] == 'A':
        # Convert speed from knots to m/s
        speed = float(parts[7]) * 0.514444  # 1 knot = 0.514444 m/s
        
        # Latitude
        lat_degrees = float(parts[3][:2]) + float(parts[3][2:]) / 60.0
        if parts[4] == 'S':
            lat_degrees = -lat_degrees
            
        latitude = lat_degrees
        
        # Longitude
        lon_degrees = float(parts[5][:3]) + float(parts[5][3:]) / 60.0
        if parts[6] == 'W':
            lon_degrees = -lon_degrees
            
        longitude = lon_degrees

    # Parse $GPGSV for number of satellites
    elif parts[0] == '$GPGSV':
        num_satellites = int(parts[3])  # Total number of satellites in view

    # Parse $GNGNS for HDOP (if available)
    elif parts[0] == '$GNGNS':
        try:
            hdop = float(parts[8])  # HDOP value
        except (IndexError, ValueError):
            hdop = None  # In case of parsing error

    return speed, latitude, longitude, num_satellites, hdop

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    print("Starting GPS data read...")
    
    speed = None
    latitude = None
    longitude = None
    num_satellites = None
    hdop = None

    try:
        while True:
            # Read a line from the GPS module
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                # Print the raw NMEA sentence for debugging
                print(f"Received: {line}")

                # Parse the line
                speed, latitude, longitude, num_satellites, hdop = parse_nmea_sentence(line)

                if speed is not None:
                    print(f"Speed: {speed:.2f} m/s")
                
                if latitude is not None and longitude is not None:
                    print(f"Latitude: {latitude:.6f}, Longitude: {longitude:.6f}")
                
                if num_satellites is not None:
                    print(f"Number of Satellites: {num_satellites}")
                
                if hdop is not None:
                    print(f"HDOP: {hdop:.2f}")
                else:
                    print("HDOP not available")

    except KeyboardInterrupt:
        print("Exiting...")
