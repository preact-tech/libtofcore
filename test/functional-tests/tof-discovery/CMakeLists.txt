#
# examples/tof-discovery
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-discovery tof-discovery.cpp)
target_link_libraries(tof-discovery tofcore Threads::Threads Boost::program_options)
