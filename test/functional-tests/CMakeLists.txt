add_subdirectory (jmp-to-boot)
add_subdirectory (rapid-changes)
add_subdirectory (simple-streamer)
add_subdirectory (tof-flip)
add_subdirectory (tof-stat)
add_subdirectory (tof-discovery)

include(GNUInstallDirs)

install(TARGETS jmp-to-boot simple-streamer tof-stat tof-discovery
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
