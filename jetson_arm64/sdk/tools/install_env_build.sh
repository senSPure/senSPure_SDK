#!/bin/sh

sudo apt update
sudo apt install libboost-all-dev=1.71.0.0ubuntu2
sudo apt install libusb-1.0-0-dev=2:1.0.23-2build1
sudo apt install build-essential cmake=3.16.3-1ubuntu1.20.04.1
sudo apt install libglfw3=3.3.2-1 libglfw3-dev=3.3.2-1
sudo apt install fonts-ipafont=00303-18ubuntu1

echo '# USB C11U Camera permission setting' > 99-usb-C11U.rules
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="14d7", ATTRS{idProduct}=="1000", GROUP="users", MODE="0777"' >> 99-usb-C11U.rules
sudo cp ./99-usb-C11U.rules /lib/udev/rules.d/
rm ./99-usb-C11U.rules

