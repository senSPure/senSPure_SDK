@echo off

rem set https_proxy={Proxy address}

choco install -y vcredist2013 vcredist140

choco install -y openssl --version 1.1.1.2100

choco install -y -s .\scripts\windows\choco_packages asio cunit eigen tinyxml-usestl tinyxml2 bullet

python -m pip install -U pip setuptools==59.6.0

python -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro

choco install -y graphviz

python -m pip install -U colcon-common-extensions

exit
