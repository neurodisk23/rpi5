from pymavlink import mavutil
import math
import statistics
import csv
import time

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

# Request specific messages at 10 Hz
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 10)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10)  # Request ATTITUDE message
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 10)

latitudes = []
longitudes = []

# Open CSV file for writing
with open('mavlink_data.csv', mode='w', newline='') as csvfile:
    fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'vx', 'vy', 'vz', 'fix_type', 'satellites_visible', 'hdop', 'roll', 'pitch', 'yaw', 'battery_voltage', 'battery_current']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    # Read messages in a loop
    while True:
        msg = connection.recv_match(blocking=True)
        if msg is not None:
            timestamp = time.time()  # Get current timestamp

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
                      f"Velocity (x): {vx}, Velocity (y): {vy}, Velocity (z): {vz}")

                # Write data to CSV
                writer.writerow({
                    'timestamp': timestamp,
                    'latitude': lat,
                    'longitude': lon,
                    'altitude': alt,
                    'vx': vx,
                    'vy': vy,
                    'vz': vz,
                    'fix_type': None,
                    'satellites_visible': None,
                    'hdop': None,
                    'roll': None,
                    'pitch': None,
                    'yaw': None,
                    'battery_voltage': None,
                    'battery_current': None
                })

            # Check for GPS_RAW_INT
            elif msg.get_type() == 'GPS_RAW_INT':
                fix_type = msg.fix_type
                satellites_visible = msg.satellites_visible
                hdop = msg.eph  # Assuming eph is the HDOP here

                print(f"GPS Fix Type: {fix_type}, Satellites Visible: {satellites_visible}, HDOP: {hdop}")

                # Update the last written row with GPS data
                if latitudes:  # Ensure we have at least one latitude entry
                    writer.writerow({
                        'timestamp': timestamp,
                        'latitude': latitudes[-1],
                        'longitude': longitudes[-1],
                        'altitude': None,
                        'vx': None,
                        'vy': None,
                        'vz': None,
                        'fix_type': fix_type,
                        'satellites_visible': satellites_visible,
                        'hdop': hdop,
                        'roll': None,
                        'pitch': None,
                        'yaw': None,
                        'battery_voltage': None,
                        'battery_current': None
                    })

            # Check for ATTITUDE
            elif msg.get_type() == 'ATTITUDE':
                roll = msg.roll * (180.0 / math.pi)  # Convert to degrees
                pitch = msg.pitch * (180.0 / math.pi)  # Convert to degrees
                yaw = msg.yaw * (180.0 / math.pi)  # Convert to degrees

                print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

                # Update the last written row with attitude data
                if latitudes:  # Ensure we have at least one latitude entry
                    writer.writerow({
                        'timestamp': timestamp,
                        'latitude': latitudes[-1],
                        'longitude': longitudes[-1],
                        'altitude': None,
                        'vx': None,
                        'vy': None,
                        'vz': None,
                        'fix_type': None,
                        'satellites_visible': None,
                        'hdop': None,
                        'roll': roll,
                        'pitch': pitch,
                        'yaw': yaw,
                        'battery_voltage': None,
                        'battery_current': None
                    })

            # Check for SYS_STATUS
            elif msg.get_type() == 'SYS_STATUS':
                print(f"Battery Voltage: {msg.voltage_battery}, Current: {msg.current_battery}")

                # Update the last written row with battery status
                if latitudes:  # Ensure we have at least one latitude entry
                    writer.writerow({
                        'timestamp': timestamp,
                        'latitude': latitudes[-1],
                        'longitude': longitudes[-1],
                        'altitude': None,
                        'vx': None,
                        'vy': None,
                        'vz': None,
                        'fix_type': None,
                        'satellites_visible': None,
                        'hdop': None,
                        'roll': None,
                        'pitch': None,
                        'yaw': None,
                        'battery_voltage': msg.voltage_battery,
                        'battery_current': msg.current_battery
                    })
