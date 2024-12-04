#
# examples/log-server
#
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost COMPONENTS program_options REQUIRED)


add_executable(log-server log-server.cpp IpClient.cpp)
target_link_libraries(log-server Threads::Threads Boost::program_options)

if (WIN32)
    set_target_properties(log-server PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
