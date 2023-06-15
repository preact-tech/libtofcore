#
# examples/tof-usb-test
#
find_package(Threads REQUIRED)

add_executable(tof-usb-test tof-usb-test.cpp)
#target_link_libraries(tof-usb-test tofcrust Threads::Threads usb-1.0 usbp-1)
target_link_libraries(tof-usb-test tofcrust Threads::Threads usbp-1)
