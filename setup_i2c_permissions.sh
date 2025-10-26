#!/bin/bash
#
# I2C Permission Setup Script for Jetson Orin Nano
# This script configures I2C permissions to allow non-root access
#

echo "=========================================="
echo "  I2C Permission Setup for Jetson Orin"
echo "=========================================="
echo

# Check if running as root
if [ "$EUID" -eq 0 ]; then
   echo "Please do NOT run as root (don't use sudo)"
   exit 1
fi

# Get current user
CURRENT_USER=$(whoami)
echo "Setting up I2C permissions for user: $CURRENT_USER"
echo

# Step 1: Add user to i2c group
echo "[1/4] Adding user to i2c group..."
sudo usermod -aG i2c $CURRENT_USER
if [ $? -eq 0 ]; then
    echo "✓ User added to i2c group"
else
    echo "✗ Failed to add user to i2c group"
    exit 1
fi
echo

# Step 2: Create udev rule
echo "[2/4] Creating udev rule..."
sudo bash -c 'cat > /etc/udev/rules.d/99-i2c.rules << EOF
# I2C device permissions
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"
EOF'

if [ $? -eq 0 ]; then
    echo "✓ udev rule created at /etc/udev/rules.d/99-i2c.rules"
else
    echo "✗ Failed to create udev rule"
    exit 1
fi
echo

# Step 3: Reload udev rules
echo "[3/4] Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
if [ $? -eq 0 ]; then
    echo "✓ udev rules reloaded"
else
    echo "✗ Failed to reload udev rules"
fi
echo

# Step 4: Check current permissions
echo "[4/4] Checking I2C device permissions..."
ls -l /dev/i2c-* 2>/dev/null
echo

# Final instructions
echo "=========================================="
echo "  Setup Complete!"
echo "=========================================="
echo
echo "IMPORTANT: You need to log out and log back in for the group"
echo "           changes to take effect."
echo
echo "After re-login, verify with:"
echo "  groups                    # Should show 'i2c' group"
echo "  i2cdetect -y -r 7         # Should work without sudo"
echo "  python3 test_i2c.py       # Should work without sudo"
echo
echo "Or run this command to apply changes immediately (in current shell):"
echo "  newgrp i2c"
echo
