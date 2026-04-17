
# microROS workspace with FreeRTOS for  Raspberry Pi Pico SDK

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Dependencies


### 0. Configure Groups
``` bash
# add user to groups
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER

# To mount PICO without sudo
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0003", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-rpi-pico.rules > /dev/null
```
(reboot your system after)

### 1. Install Pico SDK
First, make sure the Pico SDK is properly installed and configured:

```bash
# Install dependencies
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Configure environment
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```

### 2. build microROS agent (first time only)
Micro-ROS follows the client-server architecture, so you need to start the Micro-ROS Agent.

```bash
# Create a workspace and download the micro-ROS tools
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# Install pip
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Download micro-ROS agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
```

### 3. Clone this repository

Recursively clone the repo (working for `humble`).
```bash
git clone --recursive https://github.com/alexveronese/DexTrace
```

## Running Examples

#### Build all the examples

```bash
cd DexTrace
mkdir build
cd build
cmake ..
make
```

#### Flash the rehab app

To flash hold the boot button, plug the USB and run:
```bash
cp rehab.uf2 $(findmnt -rn -o TARGET -S LABEL=RPI-RP2)/
```

#### Run micro ROS agent
```bash
# Rource workspace
cd microros_ws
source install/local_setup.bash

# Run microros agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## How to use Pico SDK?

Here is a Raspberry Pi Pico C/C++ SDK documentation:
https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details. The content of this repository is derived from [micro_ros_raspberrypi_pico_sdk](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git).

For a list of other open-source components included in this repository,
see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).

## Notes
 This package is released for teaching and educational purposes only.