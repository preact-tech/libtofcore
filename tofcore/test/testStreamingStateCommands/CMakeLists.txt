set(
    TARGET 
    testStreamingStateCommands
)

add_executable(
   ${TARGET}
   ${TARGET}.cpp
)

target_link_libraries(
    ${TARGET}
    tofcore
    GTest::gmock_main
)

target_include_directories(
    ${TARGET}
    PRIVATE
    ${PROJECT_SOURCE_DIR}/tofcore/src
    ${PROJECT_SOURCE_DIR}/src
)

target_compile_features(${TARGET} PUBLIC cxx_std_17)  # Require C99 and C++17

include(GoogleTest)
gtest_discover_tests(${TARGET})
