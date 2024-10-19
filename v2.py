from pymavlink import mavutil

# Replace '/dev/ttyACM0' with your actual serial port and set the correct baud rate
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for the first heartbeat
connection.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))


# Function to request a message stream
def request_stream(message_id, frequency_hz):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1e6 / frequency_hz),  # Interval in microseconds
        0, 0, 0, 0, 0
    )

# Request specific message types at desired frequencies
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1)  # 1 Hz
request_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)           # 1 Hz

while True:
    msg = connection.recv_match(blocking=True)
    if msg is not None:
        print(msg)
