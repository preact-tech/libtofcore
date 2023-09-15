#
# examples/simple-streamer
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(simple-streamer simple-streamer.cpp)
target_link_libraries(simple-streamer tofcore Threads::Threads Boost::program_options)
