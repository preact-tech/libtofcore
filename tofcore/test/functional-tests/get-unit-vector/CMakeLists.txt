#
# get-unit-vector
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(get-unit-vector get-unit-vector.cpp)

target_link_libraries(
    get-unit-vector
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(get-unit-vector PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
