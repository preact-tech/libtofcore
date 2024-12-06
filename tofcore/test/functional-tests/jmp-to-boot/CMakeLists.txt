#
# jmp-to-boot
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(jmp-to-boot jmp-to-boot.cpp)

target_link_libraries(
    jmp-to-boot
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(jmp-to-boot PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
