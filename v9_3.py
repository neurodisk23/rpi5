from pymavlink import mavutil
import math
import statistics
import csv
import time
from RPLCD.i2c import CharLCD

lcd = CharLCD('PCF8574', 0x27)

# Write the first line
lcd.write_string('Hello, ARAI!\n')

# Pause for a moment
time.sleep(2)

# Clear the display
lcd.clear()

freq = 20

count = 0

# Establish connection
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

# Function to request data streams
def request_stream(message_id, frequency_hz):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1e6 / frequency_hz),
        0, 0, 0, 0, 0
    )

# Request specific messages at 20 Hz
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, freq)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, freq)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, freq)  # Request ATTITUDE message
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, freq)

latitudes = []
longitudes = []

# Open CSV file for writing
with open('mavlink_data.csv', mode='w', newline='') as csvfile:
    fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'vx', 'vy', 'vz', 'fix_type', 'satellites_visible', 'hdop', 'roll', 'pitch', 'yaw', 'battery_voltage', 'battery_current']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    last_write_time = time.time()  # Initialize last write time
    write_interval = 1.0 / 20  # Write data every 1/20 seconds (20 Hz)
    lcd.clear()
    # Read messages in a loop
    while True:
        
        msg = connection.recv_match(blocking=True)
        current_time = time.time()

        # Check for GLOBAL_POSITION_INT
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7  # Latitude in degrees
            lon = msg.lon / 1e7  # Longitude in degrees
            alt = msg.alt / 1000.0  # Altitude in meters
            vx = msg.vx / 100.0  # Velocity in x in m/s
            vy = msg.vy / 100.0  # Velocity in y in m/s
            vz = msg.vz / 100.0  # Velocity in z in m/s
            
            

            latitudes.append(lat)
            longitudes.append(lon)

            if len(latitudes) > 1:  # Ensure there are enough samples
                lat_stddev = statistics.stdev(latitudes)
                lon_stddev = statistics.stdev(longitudes)
                print(f"Latitude StdDev: {lat_stddev:.6f}, Longitude StdDev: {lon_stddev:.6f}")

            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, "
                  f"Velocity (x): {vx}, Velocity (y): {vy}, Velocity (z): {vz}\n")

        # Check for GPS_RAW_INT
        elif msg.get_type() == 'GPS_RAW_INT':
            fix_type = msg.fix_type
            satellites_visible = msg.satellites_visible
            hdop = msg.eph  # Assuming eph is the HDOP here

            print(f"GPS Fix Type: {fix_type}, Satellites Visible: {satellites_visible}, HDOP: {hdop}")

        # Check for ATTITUDE
        elif msg.get_type() == 'ATTITUDE':
            roll = msg.roll * (180.0 / math.pi)  # Convert to degrees
            pitch = msg.pitch * (180.0 / math.pi)  # Convert to degrees
            yaw = msg.yaw * (180.0 / math.pi)  # Convert to degrees
            

            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        # Check for SYS_STATUS
        elif msg.get_type() == 'SYS_STATUS':
            print(f"Battery Voltage: {msg.voltage_battery}, Current: {msg.current_battery}")

        # Write data to CSV at 20 Hz
        if current_time - last_write_time >= write_interval:
            # Write the latest data collected
            if latitudes:
                writer.writerow({
                    'timestamp': current_time,
                    'latitude': latitudes[-1],
                    'longitude': longitudes[-1],
                    'altitude': alt if 'alt' in locals() else None,
                    'vx': vx if 'vx' in locals() else None,
                    'vy': vy if 'vy' in locals() else None,
                    'vz': vz if 'vz' in locals() else None,
                    'fix_type': fix_type if 'fix_type' in locals() else None,
                    'satellites_visible': satellites_visible if 'satellites_visible' in locals() else None,
                    'hdop': hdop if 'hdop' in locals() else None,
                    'roll': roll if 'roll' in locals() else None,
                    'pitch': pitch if 'pitch' in locals() else None,
                    'yaw': yaw if 'yaw' in locals() else None,
                    'battery_voltage': msg.voltage_battery if 'voltage_battery' in msg else None,
                    'battery_current': msg.current_battery if 'current_battery' in msg else None
                })

            lcd.clear()
            last_write_time = current_time  # Update the last write time
            lcd.write_string(f"La: {latitudes[-1]:.3f} Lo: {latitudes[-1]:.3f} ")
            lcd.write_string(f"vx:{vx:.2f}y:{vy:.2f}z:{vz:.2f} ")
            lcd.write_string(f"R{roll:.2f}P{pitch:.2f} SV{satellites_visible:.2f}")
        
