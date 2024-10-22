from pymavlink import mavutil
import math
import statistics
import csv
import time
from RPLCD.i2c import CharLCD
import cv2
import os
from datetime import datetime

# Create a directory to save frames
output_dir = 'saved_frames'
os.makedirs(output_dir, exist_ok=True)

# Open the webcam (0 is the default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

lcd = CharLCD('PCF8574', 0x27)

# Write the first line
lcd.write_string('Hello, ARAI!\n')

# Pause for a moment
time.sleep(2)

# Clear the display
lcd.clear()

freq = 20  # Frequency for both CSV writing and frame capture
write_interval = 1.0 / freq  # 50 milliseconds
last_write_time = time.time()  # Initialize last write time
last_frame_time = last_write_time  # Initialize last frame capture time

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
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, freq)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, freq)

latitudes = []
longitudes = []

# Open CSV file for writing
with open('mavlink_data.csv', mode='w', newline='') as csvfile:
    fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'vx', 'vy', 'vz', 'fix_type', 'satellites_visible', 'hdop', 'roll', 'pitch', 'yaw', 'battery_voltage', 'battery_current']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    iteration_count = 0  # Counter for iterations
    lcd.write_string('Boot sequence start\n')
    lcd.write_string(" H A R S H A L - 1993")
    
    # Variables to store data
    last_lat = None
    last_lon = None
    last_alt = None
    last_vx = None
    last_vy = None
    last_vz = None
    last_fix_type = None
    last_satellites_visible = None
    last_hdop = None
    last_roll = None
    last_pitch = None
    last_yaw = None
    last_battery_voltage = None
    last_battery_current = None

    # Read messages in a loop
    while True:
        msg = connection.recv_match(blocking=True)
        current_time = time.time()

        # Process CSV data writing
        if current_time - last_write_time >= write_interval:
            # Check for GLOBAL_POSITION_INT
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                last_lat = msg.lat / 1e7  # Latitude in degrees
                last_lon = msg.lon / 1e7  # Longitude in degrees
                last_alt = msg.alt / 1000.0  # Altitude in meters
                last_vx = msg.vx / 100.0  # Velocity in x in m/s
                last_vy = msg.vy / 100.0  # Velocity in y in m/s
                last_vz = msg.vz / 100.0  # Velocity in z in m/s

                latitudes.append(last_lat)
                longitudes.append(last_lon)

                if len(latitudes) > 1:  # Ensure there are enough samples
                    lat_stddev = statistics.stdev(latitudes)
                    lon_stddev = statistics.stdev(longitudes)
                    print(f"Latitude StdDev: {lat_stddev:.6f}, Longitude StdDev: {lon_stddev:.6f}")

                print(f"Latitude: {last_lat}, Longitude: {last_lon}, Altitude: {last_alt}, "
                      f"Velocity (x): {last_vx}, Velocity (y): {last_vy}, Velocity (z): {last_vz}\n")

            # Check for GPS_RAW_INT
            elif msg.get_type() == 'GPS_RAW_INT':
                last_fix_type = msg.fix_type
                last_satellites_visible = msg.satellites_visible
                last_hdop = msg.eph  # Assuming eph is the HDOP here

                print(f"GPS Fix Type: {last_fix_type}, Satellites Visible: {last_satellites_visible}, HDOP: {last_hdop}")

            # Check for ATTITUDE
            elif msg.get_type() == 'ATTITUDE':
                last_roll = msg.roll * (180.0 / math.pi)  # Convert to degrees
                last_pitch = msg.pitch * (180.0 / math.pi)  # Convert to degrees
                last_yaw = msg.yaw * (180.0 / math.pi)  # Convert to degrees

                print(f"Roll: {last_roll:.2f}, Pitch: {last_pitch:.2f}, Yaw: {last_yaw:.2f}")

            # Check for SYS_STATUS
            elif msg.get_type() == 'SYS_STATUS':
                last_battery_voltage = msg.voltage_battery if 'voltage_battery' in msg else None
                last_battery_current = msg.current_battery if 'current_battery' in msg else None
                print(f"Battery Voltage: {last_battery_voltage}, Current: {last_battery_current}")

            # Write data to CSV
            writer.writerow({
                'timestamp': current_time,
                'latitude': last_lat,
                'longitude': last_lon,
                'altitude': last_alt,
                'vx': last_vx,
                'vy': last_vy,
                'vz': last_vz,
                'fix_type': last_fix_type,
                'satellites_visible': last_satellites_visible,
                'hdop': last_hdop,
                'roll': last_roll,
                'pitch': last_pitch,
                'yaw': last_yaw,
                'battery_voltage': last_battery_voltage,
                'battery_current': last_battery_current
            })

            # Update last write time
            last_write_time = current_time

            # Update the LCD every 20 iterations
            iteration_count += 1
            if iteration_count >= 20:
                lcd.clear()
                # Display relevant values on the LCD
                camera_status = 1 if current_time - last_frame_time < write_interval else 0
                lcd.write_string(f"Lat: {last_lat:.2f} Lon: {last_lon:.2f} ")
                lcd.write_string(f"SV: {last_satellites_visible} R: {last_roll:.2f} ")
                lcd.write_string(f"P: {last_pitch:.2f} Yaw: {last_yaw:.2f} ")
                lcd.write_string(f"C: {camera_status}")

                iteration_count = 0  # Reset the iteration count

        # Process frame capture
        if current_time - last_frame_time >= write_interval:
            try:
                ret, frame = cap.read()

                # Check if the frame was captured successfully
                if not ret or frame is None or frame.size == 0:
                    raise ValueError("Captured frame is invalid or empty.")

                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S') + f"_{int(datetime.now().microsecond / 1000):03d}"
                frame_filename = os.path.join(output_dir, f'frame_{timestamp}.jpg')

                # Save the current frame with timestamp
                cv2.imwrite(frame_filename, frame)
                last_frame_time = current_time  # Update last frame capture time

            except cv2.error as e:
                if "(-215:Assertion failed) !_img.empty()" in str(e):
                    print("OpenCV error: The image is empty. Skipping frame write.")
                else:
                    print(f"OpenCV error occurred: {e}")
            except ValueError as ve:
                print(f"Value error: {ve}")
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
