#
# tof-vsm
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-vsm tof-vsm.cpp)

target_link_libraries(
    tof-vsm
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(tof-vsm PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
