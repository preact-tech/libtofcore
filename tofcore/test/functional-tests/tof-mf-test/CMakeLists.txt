#
# tof-mf-test
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-mf-test tof-mf-test.cpp)

target_link_libraries(
    tof-mf-test
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(tof-mf-test PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
