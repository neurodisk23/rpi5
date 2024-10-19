from pymavlink import mavutil
import math
import statistics
import csv
import time
from RPLCD.i2c import CharLCD

lcd = CharLCD('PCF8574', 0x27)

# Write the first line
lcd.write_string('Initializing...\n')
time.sleep(2)
lcd.clear()

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
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 20)

latitudes = []
longitudes = []

# Open CSV file for writing
with open('mavlink_data.csv', mode='w', newline='') as csvfile:
    fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'vx', 'vy', 'vz', 'fix_type', 'satellites_visible', 'hdop', 'roll', 'pitch', 'yaw', 'battery_voltage', 'battery_current']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    last_write_time = time.time()
    write_interval = 1.0 / 20

    # Read messages in a loop
    while True:
        msg = connection.recv_match(blocking=True)
        current_time = time.time()

        # Initialize variables
        lat, lon, alt, vx, vy, vz, fix_type, satellites_visible, hdop, roll, pitch, yaw = (None,) * 12

        # Check for GLOBAL_POSITION_INT
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            vz = msg.vz / 100.0
            latitudes.append(lat)
            longitudes.append(lon)

            if len(latitudes) > 1:
                lat_stddev = statistics.stdev(latitudes)
                lon_stddev = statistics.stdev(longitudes)
                print(f"Latitude StdDev: {lat_stddev:.6f}, Longitude StdDev: {lon_stddev:.6f}")

            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, "
                  f"Velocity (x): {vx}, Velocity (y): {vy}, Velocity (z): {vz}")

        # Check for GPS_RAW_INT
        elif msg.get_type() == 'GPS_RAW_INT':
            fix_type = msg.fix_type
            satellites_visible = msg.satellites_visible
            hdop = msg.eph
            print(f"GPS Fix Type: {fix_type}, Satellites Visible: {satellites_visible}, HDOP: {hdop}")

        # Check for ATTITUDE
        elif msg.get_type() == 'ATTITUDE':
            roll = msg.roll * (180.0 / math.pi)
            pitch = msg.pitch * (180.0 / math.pi)
            yaw = msg.yaw * (180.0 / math.pi)
            print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

        # Check for SYS_STATUS
        elif msg.get_type() == 'SYS_STATUS':
            battery_voltage = msg.voltage_battery
            battery_current = msg.current_battery
            print(f"Battery Voltage: {battery_voltage}, Current: {battery_current}")

        # Update LCD display with latest values
        if lat is not None and lon is not None:
            lcd.clear()
            lcd.write_string(f"Lat: {lat:.2f} Lon: {lon:.2f}\n")
            lcd.write_string(f"Vx: {vx:.1f} Vy: {vy:.1f}\n")
            if roll is not None and pitch is not None and yaw is not None:
                lcd.write_string(f"R: {roll:.1f} P: {pitch:.1f} Y: {yaw:.1f}\n")
            if satellites_visible is not None:
                lcd.write_string(f"Sats: {satellites_visible}\n")

        # Write data to CSV at 20 Hz
        if current_time - last_write_time >= write_interval:
            writer.writerow({
                'timestamp': current_time,
                'latitude': lat,
                'longitude': lon,
                'altitude': alt,
                'vx': vx,
                'vy': vy,
                'vz': vz,
                'fix_type': fix_type,
                'satellites_visible': satellites_visible,
                'hdop': hdop,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'battery_voltage': battery_voltage if 'battery_voltage' in locals() else None,
                'battery_current': battery_current if 'battery_current' in locals() else None
            })
            last_write_time = current_time
