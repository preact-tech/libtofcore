# ToFCore Library
Client API for working with PreAct Time of Flight sensors 


## Requirements:

- CMake v3.16
- Requires Boost v1.70 or newer
- Python v3.8 or greater (if using python bindings)
- libudev-dev (prequisite for libusbp)
- libusbp v1.3 or newer

Libudev installation
```sudo apt-get update -y
sudo apt-get install cmake libudev-dev
```

USB Udev Rules
Create new udev rules file for usb. Example: etc/udev/rules.d/99-usb-rules.rules
```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="35FA", ATTRS{idProduct}=="0D0F", MODE:="0666"
```

## Building
 ```
 mkdir build && cd build
 cmake ..
 make -j$(nproc) 
 ``` 


## Python Bindings installation

From either the tofcore or tofcrust python wrappers directory run setup.py.
To install the python package into your personal python site-packages directory:

```
cd tofcore/wrappers/python
python3 setup.py install --user
```

## Testing
NOTE: For tests to run successfully python needs to know where to find the .so module files.
Either install the module(s) as described above or add the build location for the .so file(s) to the PYTHONPATH
environment variable. 

To run unit tests verifying behavior when no camera is connected, use the following commnad from
project's root directory: 
```
python3 -m pytest -m "not functional and not sdram_selftest" -v .
```

Functional tests with a camera connected to PC can be executed with the following command 
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
