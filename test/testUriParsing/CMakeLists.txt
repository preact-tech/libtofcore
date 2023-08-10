
add_executable(
    testUriParsing
    testUriParsing.cpp
)

target_link_libraries(
    testUriParsing
    GTest::gtest_main
)

target_include_directories(
    testUriParsing
    PRIVATE
    ${CMAKE_SOURCE_DIR}/tofcore/src
)

# target_compile_options(testUriParsing PRIVATE 
#     $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
#         -Werror -Wall -Wextra -Wconversion -Wsign-conversion -pedantic-errors>
#     $<$<CXX_COMPILER_ID:MSVC>:
#         /WX /W4>
# )

include(GoogleTest)
gtest_discover_tests(testUriParsing)

