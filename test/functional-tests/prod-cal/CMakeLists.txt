#
# examples/prod-cal
#
find_package(Threads REQUIRED)


add_executable(prod-cal prod-cal.cpp)
target_link_libraries(prod-cal tofcore Threads::Threads)
