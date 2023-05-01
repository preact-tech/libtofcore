#
# examples/jmp-to-boot
#
find_package(Threads REQUIRED)


add_executable(ib-test ib-test.cpp)
target_link_libraries(ib-test tofcore Threads::Threads)
