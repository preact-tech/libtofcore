###############################################################################
# CMakeList.txt file for the libtofcore library
# 
#  See the links below for the how and why of this CMake file structure
#  - https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/ and 
#  - https://github.com/pabloariasal/modern-cmake-sample/tree/master/libjsonutils
##
cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(libToFCore VERSION 1.2.0 LANGUAGES CXX)

#Make sure that custom modules are found
list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

include(cmake/tof_options.cmake)

###########################
# Find system dependencies
set(MIN_BOOST_VERSION 1.70)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED COMPONENTS)

find_package(usbp CONFIG QUIET)
if(NOT TARGET usbp)
    message(STATUS "libusbp not found, adding as sub_directory")
    add_subdirectory(third_party/libusbp )
endif()
###########################

add_library(tofcore)

#Add an alias so that library can be used inside the build tree, e.g. when testing
add_library(ToFCore::tofcore ALIAS tofcore)

###################
# Compiler Options:
###################
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(tofcore PUBLIC -Wall -Wextra)
endif()
if(MSVC)
    target_compile_definitions(tofcore PUBLIC WINVER=0x0A00 _WIN32_WINNT=0x0A00 -DLIBUSBP_STATIC)
endif()

target_compile_features(tofcore PUBLIC cxx_std_17)  # Require C99 and C++17

target_sources(tofcore
  PRIVATE
    src/connection.cpp
    src/comm_serial/serial_connection.cpp
    src/comm_ip/ip_connection.cpp
    src/comm_ip/tcp_connection.cpp
    src/comm_ip/udp_server.cpp
    src/device_discovery/ip_device_discovery.cpp
    src/device_discovery/usb_device_discovery.cpp
    src/device_discovery/video_device_discovery.cpp
    src/device_discovery/device_discovery.cpp
    src/measurement.cpp
    src/tof_sensor.cpp
    src/cartesian_transform.cpp
    src/crc32.cpp
    src/klv_decoder.cpp
    src/IteratorIndexIn.cpp
    src/IteratorIndexInLut.cpp
)

target_link_libraries(tofcore PRIVATE Boost::boost)
if (WIN32)
  target_link_libraries (tofcore PRIVATE iphlpapi)
endif ()
target_link_libraries(tofcore PRIVATE usbp)

#Set target properties
target_include_directories(tofcore
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/tofcore>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libusbp/include
)

include(GenerateExportHeader)
generate_export_header(tofcore)

##############################################
# Installation instructions

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/ToFCore)

if (WIN32)
    install(TARGETS tofcore
        EXPORT tofcore-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
else()
    install(TARGETS tofcore
        EXPORT tofcore-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

#This is required so that the exported target has the name ToFCore and not tofcore
set_target_properties(tofcore PROPERTIES EXPORT_NAME ToFCore)

# fixes linking issue with python bindings
set_target_properties(tofcore PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
if (WIN32)
    set_target_properties(tofcore PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

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

##############################################
## Create Package
if(UNIX)
    set(CPACK_GENERATOR "DEB")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "PreAct Tech <techsupport@preact-tech.com>")
elseif(WIN32)
    set(CPACK_GENERATOR "NSIS")
    set(CPACK_NSIS_CONTACT "PreAct Tech <techsupport@preact-tech.com>")
    set(CPACK_NSIS_MODIFY_PATH ON)
endif()

include(CPack)
##############################################
## Add test
enable_testing()
add_subdirectory(test)
add_subdirectory(wrappers)
