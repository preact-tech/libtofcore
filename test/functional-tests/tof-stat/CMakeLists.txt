#
# examples/tof-stat
#
find_package(Threads REQUIRED)


add_executable(tof-stat tof-stat.cpp)
target_link_libraries(tof-stat tofcore Threads::Threads)
