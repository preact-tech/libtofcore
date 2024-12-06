#
# tof-flip
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(tof-flip tof-flip.cpp)

target_link_libraries(
    tof-flip
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(tof-flip PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
