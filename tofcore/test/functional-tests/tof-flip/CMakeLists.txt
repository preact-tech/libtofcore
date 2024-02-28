#
# examples/tof-flip
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-flip tof-flip.cpp)
target_link_libraries(tof-flip tofcore Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(tof-flip PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
