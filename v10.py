from pymavlink import mavutil
import math
import statistics
import cv2
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
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 20)

latitudes = []
longitudes = []

# Initialize webcam
cap = cv2.VideoCapture(0)

# Read messages in a loop
while True:
    msg = connection.recv_match(blocking=True)
    if msg is not None:
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

            # Capture and save the image from the webcam
            ret, frame = cap.read()
            if ret:
                timestamp = int(time.time())
                cv2.imwrite(f'image_{timestamp}.png', frame)
                print(f"Image saved as image_{timestamp}.png")

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

# Release the webcam when done (this line will not execute in the current infinite loop)
cap.release()
cv2.destroyAllWindows()
