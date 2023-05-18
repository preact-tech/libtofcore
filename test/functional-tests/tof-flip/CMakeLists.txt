#
# examples/tof-flip
#
find_package(Threads REQUIRED)


add_executable(tof-flip tof-flip.cpp)
target_link_libraries(tof-flip tofcore Threads::Threads)
