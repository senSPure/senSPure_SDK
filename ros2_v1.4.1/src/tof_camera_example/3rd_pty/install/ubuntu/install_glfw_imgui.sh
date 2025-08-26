#!/bin/sh

sudo apt update
sudo apt install libglfw3 libglfw3-dev git

git clone https://github.com/ocornut/imgui -b v1.88
git clone https://github.com/AirGuanZ/imgui-filebrowser
cd imgui-filebrowser
git checkout cfccc2aab651cb19cbc2c3ad36be78c36078ec76
cd -

sudo apt install fonts-ipafont
