# TetraNav Proto-1 — System Test Script
# Author: Michael Tass MacDonald (Abraxas618)
# ORCID: 0009-0005-6468-7651
# Date: April 2025

import time
import smbus2
import RPi.GPIO as GPIO
from Adafruit_BNO08x import BNO08x_I2C
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

# ------------------------------
# Setup Constants
# ------------------------------
TESLA_PIN = 18  # Tesla coil control pin (PWM)

# ------------------------------
# Initialize Tesla Coil PWM
# ------------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(TESLA_PIN, GPIO.OUT)
pwm = GPIO.PWM(TESLA_PIN, 15000)  # 15kHz Tesla coil drive frequency
pwm.start(50)  # Start PWM at 50% duty cycle

# ------------------------------
# Initialize I2C Devices
# ------------------------------
i2c = smbus2.SMBus(1)

# Initialize IMU
try:
    imu = BNO08x_I2C(i2c)
    print("[✓] IMU Initialization Successful")
except Exception as e:
    print("[✗] IMU Initialization Failed:", e)

# Initialize OLED
try:
    display = Adafruit_SSD1306.SSD1306_128_64(rst=None)
    display.begin()
    display.clear()
    display.display()
    font = ImageFont.load_default()
    print("[✓] OLED Initialization Successful")
except Exception as e:
    print("[✗] OLED Initialization Failed:", e)

# ------------------------------
# Test Functions
# ------------------------------
def test_imu():
    print("\n[IMU TEST] Reading acceleration data...")
    try:
        accel = imu.acceleration
        print(f"Acceleration X: {accel[0]:.2f} m/s², Y: {accel[1]:.2f} m/s², Z: {accel[2]:.2f} m/s²")
    except Exception as e:
        print("[✗] IMU read failed:", e)

def test_oled():
    print("\n[OLED TEST] Displaying test message...")
    try:
        image = Image.new('1', (display.width, display.height))
        draw = ImageDraw.Draw(image)
        draw.text((0, 20), "TetraNav OK", font=font, fill=255)
        display.image(image)
        display.display()
        print("[✓] OLED Display Test Successful")
    except Exception as e:
        print("[✗] OLED Display Test Failed:", e)

def test_tesla():
    print("\n[TESLA COIL TEST] Pulsing Tesla Coil...")
    try:
        pwm.ChangeDutyCycle(80)
        time.sleep(0.5)
        pwm.ChangeDutyCycle(20)
        time.sleep(0.5)
        pwm.ChangeDutyCycle(50)
        print("[✓] Tesla Coil PWM Pulse Test Successful")
    except Exception as e:
        print("[✗] Tesla PWM Test Failed:", e)

# ------------------------------
# Main Test Sequence
# ------------------------------
if __name__ == "__main__":
    try:
        print("\n--- TetraNav Proto-1 System Test ---\n")
        test_imu()
        test_oled()
        test_tesla()
        print("\n--- Test Sequence Complete ---\n")
    finally:
        pwm.stop()
        GPIO.cleanup()
        try:
            display.clear()
            display.display()
        except:
            pass
