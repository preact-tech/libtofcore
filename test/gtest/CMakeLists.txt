# Use fetchcontent to pull in the gtest library
#  To use this file use add_subdirectory with the EXCLUDE_FROM_ALL flag to prevent
#  adding gtests install and test targets to the toplevel targets.
#  Example:
#    add_subdirectory(gtest EXCLUDE_FROM_ALL)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/3026483ae575e2de942db5e760cf95e973308dd5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
