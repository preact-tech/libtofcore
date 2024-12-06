#
# get-lens-info
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(get-lens-info get-lens-info.cpp)

target_link_libraries(
    get-lens-info
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(get-lens-info PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
