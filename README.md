# LibToFCore

Libraries to interface with the PreAct TOF sensors.

## Environment Setup
Requirements:

- CMake v3.16
- Requires Boost v1.70 or newer
- Python v3.8 or greater (if using python bindings)
- Python setuptools
- libudev-dev (prequisite for libusbp)
- libusbp v1.3 or newer

Python setuptools installation
(might need Python venv setup)
```
sudo pip install setuptools
```

Libudev installation
```
sudo apt-get update -y
sudo apt-get install cmake libudev-dev pkg-config g++
```

USB Udev Rules

Create new udev rules file for usb. Example: `etc/udev/rules.d/99-usb-rules.rules`
```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="35FA", ATTRS{idProduct}=="0D0F", MODE:="0666"
```

## Build and Install

### Build
```
cmake -B build
cmake --build build
```

### Install to local system
```
cmake --build build -- install  # Installs to /usr/local on UNIX systems
```

## Python Bindings Installation

To install the python package into your personal python site-packages directory:

```
make pytofcore
make pytofcrust
```

## Testing

### Unit Tests
To run unit tests verifying behavior when no camera is connected, use the following commnad from
project's root directory: 
```
python3 -m pytest -m "not functional and not sdram_selftest" -v .
```

### Functional Tests
Functional tests with a camera connected to PC can be executed with the following commands:

#### For variants without ethernet capabilities
```
python3 -m pytest -m "functional" -k "not test_ip_measurement_endpoint" -v .
```

#### For variants with ethernet
```
python3 -m pytest -m "functional" -v .
```

#### SDRAM self-test
NOTE: connection with the device will be lost, as the device will reset:
```
python3 -m pytest -m "sdram_selftest" -v .
```

#### Optional: Specific URI
For connecting to a specific connected device.

##### Examples

Linux serial type device
```
python3 -m pytest -m "functional" -v . --sensor-uri=tofserial:/dev/ttyACM12?baudrate=115200
```

Windows COM port
```
python3 -m pytest -m "functional" -v . --sensor-uri=tofserial:COM1
```

IP network device
```
python3 -m pytest -m "functional" -v . --sensor-uri=tofnet:10.10.31.180
```
