import serial
import time

# Configure the serial connection
serial_port = '/dev/ttyAMA0'  # Adjust this to your port
baud_rate = 9600                # Common baud rate for GPS modules

# Function to parse GPS data
def parse_nmea_sentence(sentence):
    parts = sentence.split(',')
    if parts[0] == '$GPRMC' and parts[2] == 'A':
        # Extract latitude and longitude from $GPRMC
        latitude = float(parts[3]) / 100  # Convert to decimal
        lat_direction = parts[4]
        longitude = float(parts[5]) / 100  # Convert to decimal
        long_direction = parts[6]
        
        # Convert to decimal degrees
        lat_degrees = int(latitude)
        lat_minutes = (latitude - lat_degrees) * 100 / 60
        lat_decimal = lat_degrees + lat_minutes
        if lat_direction == 'S':
            lat_decimal *= -1
        
        long_degrees = int(longitude)
        long_minutes = (longitude - long_degrees) * 100 / 60
        long_decimal = long_degrees + long_minutes
        if long_direction == 'W':
            long_decimal *= -1
        
        return lat_decimal, long_decimal
    
    return None, None

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
    print("Starting GPS data read...")
    
    try:
        while True:
            # Read a line from the GPS module
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                #print(line)  # Print the NMEA sentence
                
                # Parse for latitude and longitude
                lat, long = parse_nmea_sentence(line)
                if lat is not None and long is not None:
                    print(f"Latitude: {lat}, Longitude: {long}")
                    
    except KeyboardInterrupt:
        print("Exiting...")
