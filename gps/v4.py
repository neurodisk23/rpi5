import time
import board
import busio
from adafruit_bno055 import BNO055_I2C

# Initialize I2C with alternate GPIO pins
i2c = busio.I2C(board.D5, board.D4)  # SCL -> GPIO 5, SDA -> GPIO 4
bno = BNO055_I2C(i2c)

# Calibration check
def check_calibration():
    sys, gyro, accel, mag = bno.calibration_status
    print(f"Calibration - Sys: {sys}, Gyro: {gyro}, Accel: {accel}, Mag: {mag}")

while True:
    check_calibration()
    
    # Read Euler angles
    euler = bno.euler
    print(f"Euler Angles: {euler}")

    # Read Quaternion
    quaternion = bno.quaternion
    print(f"Quaternion: {quaternion}")

    # Delay for readability
    time.sleep(1)
