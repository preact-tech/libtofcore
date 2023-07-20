#
# examples/tof-lens-rd
#
find_package(Threads REQUIRED)


add_executable(tof-lens-rd tof-lens-rd.cpp)
target_link_libraries(tof-lens-rd tofcore Threads::Threads)
