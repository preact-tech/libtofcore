###############################################################################
# CMakeList.txt file for the libtofcore library
# 
#  See the links below for the how and why of this CMake file structure
#  - https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/ and 
#  - https://github.com/pabloariasal/modern-cmake-sample/tree/master/libjsonutils
##
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(libtToFCore VERSION 1.0.0 LANGUAGES CXX)

#Make sure that custom modules are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

include(cmake/tof_options.cmake)

###########################
# Find system dependencies
set(MIN_BOOST_VERSION 1.70)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED COMPONENTS)
###########################

add_library(tofcore)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(ToFCore::tofcore ALIAS tofcore)

###################
# Compiler Options:
###################
target_compile_options(tofcore PUBLIC -Wall -Wextra -fPIC -g)
target_compile_features(tofcore PUBLIC cxx_std_17)  # Require C99 and C++17


target_sources(tofcore
  PRIVATE
    src/comm_serial/serial_connection.cpp
    src/measurement.cpp
    src/tof_sensor.cpp
    src/cartesian_transform.cpp
    src/comm_ethernet/tcp_connection.cpp
    src/comm_ethernet/udp_server.cpp
    src/crc32.cpp
    src/klv_decoder.cpp
)

#Set target properties
target_include_directories(tofcore
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tofcore>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


target_link_libraries(tofcore
  PUBLIC
  PRIVATE
  # Boost::regex example of private dependency
)

include(GenerateExportHeader)
generate_export_header(tofcore)

##############################################
# Installation instructions

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ToFCore)

install(TARGETS tofcore
    EXPORT tofcore-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

#This is required so that the exported target has the name ToFCore and not tofcore
set_target_properties(tofcore PROPERTIES EXPORT_NAME ToFCore)

# fixes linking issue with python bindings
set_target_properties(tofcore PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/tofcore_export.h" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/tofcore")

#Export the targets to a script
install(EXPORT tofcore-targets
    FILE
        ToFCoreTargets.cmake
    NAMESPACE
        ToFCore::
    DESTINATION
        ${INSTALL_CONFIGDIR}
)

#Create a ConfigVersion.cmake filef
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCoreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/ToFCoreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCoreConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

#Install the config, configversion and custom find modules
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCoreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/ToFCoreConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
## Exporting from the build tree
export(EXPORT tofcore-targets
    FILE ${CMAKE_CURRENT_BINARY_DIR}/ToFCoreTargets.cmake
    NAMESPACE ToFCore::)

#Register package in the User Package Registry
export(PACKAGE ToFCore)

###############################################
## Python bindings
if(BUILD_PYTHON_BINDINGS)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG        v2.10.4
    )

    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(pytofcore src/py_tofcore.cpp)
    target_link_libraries(pytofcore PRIVATE tofcore)

    target_compile_definitions(pytofcore
                            PRIVATE VERSION_INFO=${VERSION_PACKAGE})
endif(BUILD_PYTHON_BINDINGS)

##############################################
## Add test
enable_testing()
add_subdirectory(test)
