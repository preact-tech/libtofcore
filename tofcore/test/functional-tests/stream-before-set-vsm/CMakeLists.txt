#
# stream-before-set-vsm
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(stream-before-set-vsm stream-before-set-vsm.cpp)

target_link_libraries(
    stream-before-set-vsm
    tofcore 
    Threads::Threads 
    Boost::program_options
    dbg_out
    po_count
)

if (WIN32)
    set_target_properties(stream-before-set-vsm PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
