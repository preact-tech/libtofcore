#
# examples/tof-mf-test
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(tof-mf-test tof-mf-test.cpp)
target_link_libraries(tof-mf-test tofcore Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(tof-mf-test PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
