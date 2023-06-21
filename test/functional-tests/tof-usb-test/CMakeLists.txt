#
# examples/tof-usb-test
#
find_package(Threads REQUIRED)

add_executable(tof-usb-test tof-usb-test.cpp)
target_link_libraries(tof-usb-test tofcore Threads::Threads usbp-1)
