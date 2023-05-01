#
# examples/sequ-ctrl
#
find_package(Threads REQUIRED)


add_executable(sequ-ctrl sequ-ctrl.cpp)
target_link_libraries(sequ-ctrl tofcore Threads::Threads)
