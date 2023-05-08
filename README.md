# LibToFCore

Library to interface with the PreAct TOF sensors.

# Quick start

Requirements: 

- CMake v3.16
- Requires Boost v1.70 or newer
- Python v3.8 or greater (if using python bindings)

Normal build and install of library:

```bash
cmake -Bbuild
cmake --build build
```

or 

```
python3 setup.py build
```

Normal installation:

```bash
sudo cmake --build build --install  # Installs to /usr/local on UNIX systems
```

_See [CMake documentaion](https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX.html) on default install location and how to change it._

## Python Bindings installation

From wrappers/python directory run setup.py: 

```
cd tofcore/wrappers/python
python3 setup.py install --user
```

This will install the python package `pytofcore` in your personal python site-packages directory. 


# Testing
NOTE: For tests to run successfully python needs to know where to find the `pytofcore.xxx.so` file. 
Either install the module as described above or add the build location for the .so file(s) to the PYTHONPATH
environment variable. 

To run unit tests, use the following commnad from project's root directory: 
```
python3 -m pytest -m "not functional" -v .
```

Those set of tests can run when there is no camera connected to PC. 

Functional tests with a camera connected to PC can be executed with the following command 
```
python3 -m pytest -m "functional" -v .
```

Optionally specify the port (device node) to connect to the sensor on with the option e.g. `--sensor-port-name=/dev/ttyACM#`  or `COM1`
