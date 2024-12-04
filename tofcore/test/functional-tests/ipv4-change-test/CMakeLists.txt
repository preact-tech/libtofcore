#
# examples/ipv4-change-test
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(ipv4-change-test ipv4-change-test.cpp)

target_link_libraries(
    ipv4-change-test
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(ipv4-change-test PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
