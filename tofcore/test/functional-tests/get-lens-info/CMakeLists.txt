#
# examples/get-lens-info
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(get-lens-info get-lens-info.cpp)
target_link_libraries(get-lens-info tofcore Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(get-lens-info PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
