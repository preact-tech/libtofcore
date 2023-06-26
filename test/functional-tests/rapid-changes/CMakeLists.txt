#
# examples/rapid-changes
#
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

add_executable(rapid-changes rapid-changes.cpp)
target_link_libraries(rapid-changes tofcore Threads::Threads Boost::program_options)
