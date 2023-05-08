cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(ToFCorePythonWrapper)

include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.10.4
)

FetchContent_MakeAvailable(pybind11)

pybind11_add_module(pytofcore py_tofcore.cpp)
target_link_libraries(pytofcore PRIVATE tofcore)

target_compile_definitions(pytofcore
                        PRIVATE VERSION_INFO=${VERSION_PACKAGE})
