#!/bin/sh

sudo apt update
sudo apt install libboost-all-dev
sudo apt install libusb-1.0-0-dev
sudo apt install build-essential cmake
sudo apt install libglfw3 libglfw3-dev
sudo apt install fonts-ipafont

echo '# USB C11U Camera permission setting' > 99-usb-C11U.rules
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="14d7", ATTRS{idProduct}=="1000", GROUP="users", MODE="0777"' >> 99-usb-C11U.rules
sudo cp ./99-usb-C11U.rules /lib/udev/rules.d/
rm ./99-usb-C11U.rules

