#!/bin/bash

# TetraNav Proto-1 Install Script
# Author: Michael Tass MacDonald (Abraxas618)
# April 2025

echo "🚀 Starting TetraNav Proto-1 Installation..."

# -------------------------
# Update + Upgrade
# -------------------------
echo "🔄 Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# -------------------------
# Install Python and PIP
# -------------------------
echo "🐍 Installing Python3 and pip..."
sudo apt-get install -y python3 python3-pip python3-smbus i2c-tools

# -------------------------
# Enable I2C Interface
# -------------------------
echo "🛠️ Enabling I2C hardware support..."
sudo raspi-config nonint do_i2c 0

# -------------------------
# Install Python Libraries
# -------------------------
echo "📦 Installing required Python libraries..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

# -------------------------
# Create Folders
# -------------------------
echo "📂 Creating telemetry and hardware directories..."
mkdir -p telemetry
mkdir -p hardware

# -------------------------
# Set Permissions
# -------------------------
chmod +x tetranav_main.py
chmod +x test_tetranav.py

# -------------------------
# Final Message
# -------------------------
echo "✅ TetraNav Proto-1 installation complete!"
echo "🔌 Please reboot your Raspberry Pi to finalize I2C settings."
echo "After reboot, run:"
echo "    python3 test_tetranav.py"
echo "to verify system health."

exit 0
