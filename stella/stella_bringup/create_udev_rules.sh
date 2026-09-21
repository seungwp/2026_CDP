#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "remap the devices serial port(ttyUSBX, ttySX) to AHRS, Motordriver, Bluetooth"
echo "devices usb connection as /dev/AHRS, /dev/MW, /dev/BT, check it using the command : ls -l /dev|grep -e ttyUSB -e ttyS0"
echo "start copy stella.rules to  /etc/udev/rules.d/"
echo "$SCRIPT_DIR/stella.rules"
sudo cp "$SCRIPT_DIR/stella.rules" /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm trigger
echo "finish "
