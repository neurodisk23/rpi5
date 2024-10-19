from pymavlink import mavutil

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
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 20)

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

            print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}, "
                  f"Velocity (x): {vx}, Velocity (y): {vy}, Velocity (z): {vz}")

        # Check for GPS_RAW_INT
        elif msg.get_type() == 'GPS_RAW_INT':
            fix_type = msg.fix_type
            satellites_visible = msg.satellites_visible
            hdop = msg.eph  # Assuming eph is the HDOP here

            print(f"GPS Fix Type: {fix_type}, Satellites Visible: {satellites_visible}, HDOP: {hdop}")

        # Check for SYS_STATUS
        elif msg.get_type() == 'SYS_STATUS':
            print(f"Battery Voltage: {msg.voltage_battery}, Current: {msg.current_battery}")
