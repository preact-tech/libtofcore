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

At project's root directory, run:

```
 pip install --user .
```

This will install the python package `pytofcore` in your python site-packages directory. 


# Testing
NOTE: For tests to run successfully, create a symbolic link to `pytofcore.xxx.so` inside `tests` directory. The `.so` file is in build directory. Alternatively, you can install the `pytofcore` package using `pip`

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
