#
# libpo_count
#

add_library (
    po_count
    INTERFACE
)

target_include_directories (
    po_count
    INTERFACE .
)

if (WIN32)
    set_target_properties(po_count PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()
