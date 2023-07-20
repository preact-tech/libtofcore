#
# examples/get-lens-info
#
find_package(Threads REQUIRED)


add_executable(get-lens-info get-lens-info.cpp)
target_link_libraries(get-lens-info tofcore Threads::Threads)
