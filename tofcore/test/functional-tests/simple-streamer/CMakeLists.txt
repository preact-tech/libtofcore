#
# examples/simple-streamer
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(simple-streamer simple-streamer.cpp)
target_link_libraries(simple-streamer tofcore Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(simple-streamer PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
