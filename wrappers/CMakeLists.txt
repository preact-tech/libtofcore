project(ToFEngWrappers)

set(DEPENDENCIES tofeng)

if (BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)        
endif()