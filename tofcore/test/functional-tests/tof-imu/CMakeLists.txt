#
# tof-imu
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-imu tof-imu.cpp)

target_link_libraries(
    tof-imu
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(tof-imu PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
