#
# examples/tof-discovery
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS        ON)  # only find static libs
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-discovery tof-discovery.cpp)
target_link_libraries(tof-discovery tofcore Threads::Threads Boost::program_options)
