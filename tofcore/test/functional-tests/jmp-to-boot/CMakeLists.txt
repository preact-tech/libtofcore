#
# examples/jmp-to-boot
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(jmp-to-boot jmp-to-boot.cpp)
target_link_libraries(jmp-to-boot tofcore Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(jmp-to-boot PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
