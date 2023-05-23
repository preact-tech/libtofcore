#
# examples/rapid-changes
#
find_package(Threads REQUIRED)


add_executable(rapid-changes rapid-changes.cpp)
target_link_libraries(rapid-changes tofcore Threads::Threads)
