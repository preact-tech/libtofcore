add_executable(testUriParsing
    testUriParsing.cpp
)

target_link_libraries(testUriParsing
    PRIVATE
        GTest::gtest_main
        GTest::gtest
        ToFCore::tofcore
)

target_include_directories(testUriParsing
    PRIVATE
        ${PROJECT_SOURCE_DIR}/tofcore/src
        ${PROJECT_SOURCE_DIR}/src
)

include(GoogleTest)

if (WIN32)
    add_custom_command(
        TARGET testUriParsing
        POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:testUriParsing> $<TARGET_FILE_DIR:testUriParsing>
            COMMAND_EXPAND_LISTS
    )
endif()

gtest_discover_tests(testUriParsing)
