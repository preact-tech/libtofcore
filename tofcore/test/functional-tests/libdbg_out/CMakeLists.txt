#
# libdbg_out
#

add_library (
    dbg_out
    INTERFACE
)

target_include_directories (
    dbg_out
    INTERFACE .
)

if (WIN32)
    set_target_properties(dbg_out PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
