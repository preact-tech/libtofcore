# Windows build

## Building in docker container
### Build and launch docker container

It is recommended to use the `OasisWinDev.bat` tool to create and use a docker development environment. [Click here for more details](https://bitbucket.org/preact-tech/oasis-win-dev/src/develop/). e.g.:

```batch
OasisWinDev.bat aws-login
OasisWinDev.bat image-pull
OasisWinDev.bat run
```

### Building libtofcore

```
cmake -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake `
                    -DVCPKG_TARGET_TRIPLET=x64-windows-static `
                    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded `
                    -DBoost_NO_WARN_NEW_VERSIONS=1 `
                    -DCMAKE_INSTALL_PREFIX=dist/win `
                    -DVERSION_PACKAGE="0.0.0" `
                    -B build-win -S .
cmake --build build-win --config release --target install
```

The resulting test applications, libraries and client application headers will be place in the `dist\win` directory. 
The Python3.11 .pyd modules will be in `build-win\pytof[core|crust]\wrappers\python`

## Building without Docker

An install of Visual Studio 2022 is required along with the `vcpkg` package manager. 

Launch A Developer COmmand Prompt for VS 2022

```
cmake -DCMAKE_TOOLCHAIN_FILE=<PATH-TO-VCPKG-INSTALL>/scripts/buildsystems/vcpkg.cmake ^
                    -DVCPKG_TARGET_TRIPLET=x64-windows-static ^
                    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
                    -DBoost_NO_WARN_NEW_VERSIONS=1 ^
                    -DCMAKE_INSTALL_PREFIX=dist/win ^
                    -DVERSION_PACKAGE="0.0.0" ^
                    -B build -S .
```

Visual Studio solution and project files will be written to the `build` directory and can be opened
in the Visual Studio IDE and or built using MSBuild or cmake directly. 

