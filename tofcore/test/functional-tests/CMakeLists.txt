add_subdirectory (libdbg_out)
add_subdirectory (libpo_count)
add_subdirectory (get-lens-info)
add_subdirectory (get-unit-vector)
add_subdirectory (ipv4-change-test)
add_subdirectory (jmp-to-boot)
add_subdirectory (log-server)
add_subdirectory (rapid-changes)
add_subdirectory (simple-streamer)
add_subdirectory (stream-before-set-vsm)
add_subdirectory (tof-discovery)
add_subdirectory (tof-edge)
add_subdirectory (tof-flip)
add_subdirectory (tof-imu)
add_subdirectory (tof-ipv4)
add_subdirectory (tof-mf-test)
add_subdirectory (tof-stat)
add_subdirectory (tof-vsm)
add_subdirectory (vsm-streamer)

include(GNUInstallDirs)

install(TARGETS
    get-lens-info
    get-unit-vector
    ipv4-change-test
    jmp-to-boot
    log-server
    rapid-changes
    simple-streamer
    stream-before-set-vsm
    tof-discovery
    tof-edge
    tof-flip
    tof-imu
    tof-ipv4
    tof-mf-test
    tof-stat
    tof-vsm
    vsm-streamer
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
