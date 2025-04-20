# TetraNav Proto-1 — Real-Time OLED Dashboard
# Author: Michael Tass MacDonald (Abraxas618)
# ORCID: 0009-0005-6468-7651
# Date: April 2025

import time
import csv
import os
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

# ------------------------------
# Constants
# ------------------------------
TELEMETRY_FILE = "telemetry/tetranav_telemetry.csv"  # Path to telemetry log
REFRESH_RATE = 1.0  # OLED refresh every 1 second

# ------------------------------
# Initialize OLED Display
# ------------------------------
try:
    display = Adafruit_SSD1306.SSD1306_128_64(rst=None)
    display.begin()
    display.clear()
    display.display()
    font = ImageFont.load_default()
    print("[✓] OLED Dashboard Initialized")
except Exception as e:
    print("[✗] OLED Initialization Failed:", e)
    exit(1)

# ------------------------------
# Read Last Line of Telemetry File
# ------------------------------
def read_latest_telemetry(filepath):
    if not os.path.exists(filepath):
        return None

    try:
        with open(filepath, "r") as f:
            lines = f.readlines()
            if len(lines) < 2:
                return None  # No data yet
            last_line = lines[-1]
            parts = last_line.strip().split(",")
            if len(parts) >= 7:
                data = {
                    "timestamp": parts[0][-8:],  # Show only HH:MM:SS
                    "pos_x": float(parts[1]),
                    "pos_y": float(parts[2]),
                    "pos_z": float(parts[3]),
                    "drift_x": float(parts[4]),
                    "drift_y": float(parts[5]),
                    "drift_z": float(parts[6])
                }
                return data
    except Exception as e:
        print("Error reading telemetry:", e)
    return None

# ------------------------------
# Main Dashboard Loop
# ------------------------------
try:
    while True:
        telemetry = read_latest_telemetry(TELEMETRY_FILE)
        
        image = Image.new('1', (display.width, display.height))
        draw = ImageDraw.Draw(image)
        
        if telemetry:
            draw.text((0, 0), f"TetraNav - {telemetry['timestamp']}", font=font, fill=255)
            draw.text((0, 16), f"X: {telemetry['pos_x']:.2f}", font=font, fill=255)
            draw.text((0, 26), f"Y: {telemetry['pos_y']:.2f}", font=font, fill=255)
            draw.text((0, 36), f"Z: {telemetry['pos_z']:.2f}", font=font, fill=255)
            draw.text((0, 50), f"D: {telemetry['drift_x']:.1f},{telemetry['drift_y']:.1f}", font=font, fill=255)
        else:
            draw.text((0, 0), "Waiting for", font=font, fill=255)
            draw.text((0, 12), "telemetry data...", font=font, fill=255)

        display.image(image)
        display.display()

        time.sleep(REFRESH_RATE)

except KeyboardInterrupt:
    print("\n[INFO] Dashboard Shutdown")
    display.clear()
    display.display()
