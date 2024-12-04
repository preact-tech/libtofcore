#
# tof-ipv4
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-ipv4 tof-ipv4.cpp)

target_link_libraries(
    tof-ipv4
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(tof-ipv4 PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
