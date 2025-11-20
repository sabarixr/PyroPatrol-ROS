#!/bin/bash

# Fire Fighter Rover - Fix Permissions Script
echo "🔧 Fire Fighter Rover - Setting up permissions..."

# Add user to gpio and i2c groups
echo "👤 Adding user to hardware groups..."
sudo usermod -a -G gpio,i2c,video $USER

# Set GPIO permissions for non-root access
echo "🔌 Setting up GPIO permissions..."
if [ ! -f /etc/udev/rules.d/99-gpio.rules ]; then
    echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c '\''chown root:gpio /sys/class/gpio/export /sys/class/gpio/unexport; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'\''"' | sudo tee /etc/udev/rules.d/99-gpio.rules
    echo 'SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c '\''chown root:gpio /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'\''"' | sudo tee -a /etc/udev/rules.d/99-gpio.rules
fi

# Set I2C permissions
echo "📡 Setting up I2C permissions..."
if [ ! -f /etc/udev/rules.d/99-i2c.rules ]; then
    echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee /etc/udev/rules.d/99-i2c.rules
fi

# Set video permissions
echo "📹 Setting up camera permissions..."
if [ ! -f /etc/udev/rules.d/99-camera.rules ]; then
    echo 'KERNEL=="video[0-9]*", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/99-camera.rules
fi

# Alternative: Install pigpio for GPIO access (more reliable)
echo "🐷 Installing pigpio for better GPIO access..."
sudo apt update
sudo apt install -y pigpio python3-pigpio

# Enable pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Install missing Python packages
echo "🐍 Installing missing Python packages..."
pip3 install --user flask

# Add local bin directory to PATH
echo "🛤️  Adding ~/.local/bin to PATH..."
if ! grep -q '$HOME/.local/bin' ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    echo "   Added to ~/.bashrc"
else
    echo "   Already in ~/.bashrc"
fi

echo ""
echo "✅ Permissions setup complete!"
echo ""
echo "⚠️  IMPORTANT: You must REBOOT or LOGOUT/LOGIN for group changes to take effect!"
echo ""
echo "🔄 After reboot, test with:"
echo "   ./test_system.sh"
echo ""
echo "🚀 Then launch with:"
echo "   ros2 launch frr_bringup rover_bringup.launch.py"
