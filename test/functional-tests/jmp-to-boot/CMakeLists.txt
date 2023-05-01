#
# examples/jmp-to-boot
#
find_package(Threads REQUIRED)


add_executable(jmp-to-boot jmp-to-boot.cpp)
target_link_libraries(jmp-to-boot tofcore Threads::Threads)
