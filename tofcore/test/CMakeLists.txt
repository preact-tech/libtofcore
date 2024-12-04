find_package(GTest CONFIG
    COMPONENTS
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)
if ((NOT TARGET GTest::gtest) OR (NOT TARGET GTest::gmock))
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/3026483ae575e2de942db5e760cf95e973308dd5.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_GetProperties(googletest)
    # Use this instead of FetchContent_MakeAvailable to prevent installation of
    # GTest. We don't install the test target, therefore, there is no need to
    # install GTest.
    # Note that with CMake 3.28 or newer you can simply add EXCLUDE_FROM_ALL to
    # FetchContent_Declare and use FetchContent_MakeAvailable.
    if (NOT googletest_POPULATED)
        FetchContent_Populate(googletest)
        add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
endif()


add_subdirectory (testStreamingStateCommands)

add_subdirectory (functional-tests)
add_subdirectory (testUriParsing)
