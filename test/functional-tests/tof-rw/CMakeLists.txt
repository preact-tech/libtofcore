#
# examples/tof-rw
#
find_package(Threads REQUIRED)


add_executable(tof-rw tof-rw.cpp)
target_link_libraries(tof-rw tofcore Threads::Threads)
target_include_directories(tof-rw PRIVATE ../../../src)
