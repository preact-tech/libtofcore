#
# examples/simple-streamer
#
find_package(Threads REQUIRED)


add_executable(simple-streamer simple-streamer.cpp)
target_link_libraries(simple-streamer tofcore Threads::Threads)
