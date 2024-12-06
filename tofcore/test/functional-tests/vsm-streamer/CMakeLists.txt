#
# vsm-streamer
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(vsm-streamer vsm-streamer.cpp)

target_link_libraries(
    vsm-streamer
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(vsm-streamer PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
