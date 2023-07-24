#
# examples/tof-flip
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-flip tof-flip.cpp)
target_link_libraries(tof-flip tofcore Threads::Threads Boost::program_options)
