# LibToFCore

Libraries to interface with the PreAct TOF sensors.

tofcore: For basic average user access to TOF sensors.
tofcrust: For advanced engineering/production access to TOF sensors. 

_Why tofcrust? because it's a crusty wrapper round the core ;-P (you can thank Lance)_

# Quick start

The easiest method to ensure all the needed tools of the correct versions are 
present is to use the oasis-dev Docker container.

## Rolling your own environment
Requirements: 

- CMake v3.16
- Requires Boost v1.70 or newer
- Python v3.8 or greater (if using python bindings)
- Python setuptools
- libudev-dev (prequisite for libusbp)
- libusbp v1.3 or newer

Python setuptools installation
```
sudo -m pip install setuptools
```

Libudev installation
```
sudo apt-get update -y
sudo apt-get install cmake libudev-dev pkg-config g++
```

USB Udev Rules
Create new udev rules file for usb. Example: etc/udev/rules.d/99-usb-rules.rules
```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="35FA", ATTRS{idProduct}=="0D0F", MODE:="0666"
```

## Normal build and install of library:

```
bash
make build
```

## Python Bindings installation

To install the python package into your personal python site-packages directory:

```
make pytofcore
make pytofcrust
```

# Testing

To run unit tests verifying behavior when no camera is connected, use the following commnad from
project's root directory: 
```
python3 -m pytest -m "not functional and not sdram_selftest" -v .
```

Functional tests with a camera connected to PC can be executed with the following commands:

For variants without ethernet capabilities

```
python3 -m pytest -m "functional" -k "not test_ip_measurement_endpoint" -v .
```

For variants with ethernet

```
python3 -m pytest -m "functional" -v .
```

To run the SDRAM self-test (NOTE: connection with the device will be lost, as the device will reset):
```
python3 -m pytest -m "sdram_selftest" -v .
```

Optionally specify the URI for connecting to a specific connected device:
Examples:
```
# Linux serial type device
python3 -m pytest -m "functional" -v . --sensor-uri=tofserial:/dev/ttyACM12?baudrate=115200

# Windows COM port
python3 -m pytest -m "functional" -v . --sensor-uri=tofserial:COM1

# IP network device
python3 -m pytest -m "functional" -v . --sensor-uri=tofnet:10.10.31.180
```
