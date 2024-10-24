import serial
import time
import adafruit_character_lcd.character_lcd_i2c as character_lcd
import board
import busio

# Set up serial connection for GPS
gps_port = '/dev/serial0'  # Use /dev/serial0 on Raspberry Pi
gps_serial = serial.Serial(gps_port, baudrate=9600, timeout=1)

# Set up I2C for LCD
i2c = busio.I2C(board.SCL, board.SDA)
lcd_columns = 20
lcd_rows = 4
lcd = character_lcd.Character_LCD_I2C(i2c, lcd_columns, lcd_rows)

# Function to read GPS data
def read_gps():
    while True:
        if gps_serial.in_waiting > 0:
            data = gps_serial.readline().decode('ascii', errors='replace')
            if data.startswith('$GPGGA'):
                print(data.strip())  # Print to terminal
                lcd.clear()
                lcd.message = data.strip()[:20]  # Display on LCD (limit to 20 characters)
                time.sleep(1)  # Delay for readability

try:
    print("Starting GPS reader...")
    lcd.message = "GPS Initialized"
    time.sleep(2)  # Show message for a bit
    lcd.clear()
    read_gps()
except KeyboardInterrupt:
    print("Program stopped.")
finally:
    gps_serial.close()
