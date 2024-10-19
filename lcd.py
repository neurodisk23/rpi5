import time
from RPLCD.i2c import CharLCD

# Change '0x27' to your I2C address if different
lcd = CharLCD('PCF8574', 0x27)

lcd.write_string('Hello, World!')
time.sleep(2)

lcd.clear()
lcd.write_string('Raspberry Pi')
