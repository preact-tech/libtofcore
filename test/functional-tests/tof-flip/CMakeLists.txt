#
# examples/tof-flip
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-flip tof-flip.cpp)
target_link_libraries(tof-flip tofcore Threads::Threads Boost::program_options)
