add_subdirectory (get-lens-info)
add_subdirectory (jmp-to-boot)
add_subdirectory (rapid-changes)
add_subdirectory (simple-streamer)
add_subdirectory (tof-discovery)
add_subdirectory (tof-flip)
add_subdirectory (tof-ipv4)
add_subdirectory (tof-mf-test)
add_subdirectory (tof-stat)
add_subdirectory (tof-vsm)
add_subdirectory (vsm-streamer)

include(GNUInstallDirs)

install(TARGETS
    get-lens-info
    jmp-to-boot
    rapid-changes
    simple-streamer
    tof-discovery
    tof-flip
    tof-ipv4
    tof-mf-test
    tof-stat
    tof-vsm
    vsm-streamer
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
