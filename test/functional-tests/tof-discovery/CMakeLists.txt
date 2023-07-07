#
# examples/tof-discovery
#
find_package(Threads REQUIRED)

add_executable(tof-discovery tof-discovery.cpp)
target_link_libraries(tof-discovery tofcore Threads::Threads)
