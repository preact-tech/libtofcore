#
# examples/tof-stat
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-stat tof-stat.cpp)
target_link_libraries(tof-stat tofcore Threads::Threads Boost::program_options)
