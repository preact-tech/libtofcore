REM This is a quick and dirty batch script for building on a windows system with VS 2022. 
REM  it work on my system but the paths maybe incorrect or ...

IF "%1"=="1" cmake -DCMAKE_TOOLCHAIN_FILE="c:/Program Files/Microsoft Visual Studio\2022/Community/VC/vcpkg/scripts/buildsystems/vcpkg.cmake" ^
                    -DVCPKG_TARGET_TRIPLET=x64-windows-static ^
                    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded ^
                    -DBoost_NO_WARN_NEW_VERSIONS=1 ^
                    -DCMAKE_INSTALL_PREFIX=dist/win ^
                    -DVERSION_PACKAGE="0.0.0" ^
                    -DPYBIND11_PYTHON_VERSION="3.8" ^
                    -DPYTHON_EXECUTABLE="C:/Program Files/python38/python.exe" ^
                    -Wno-dev ^
                    -B build -S .

cmake --build build --config release --target install
