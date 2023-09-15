add_subdirectory (get-lens-info)
add_subdirectory (jmp-to-boot)
add_subdirectory (rapid-changes)
add_subdirectory (simple-streamer)
add_subdirectory (tof-discovery)
add_subdirectory (tof-flip)
add_subdirectory (tof-ipv4)
add_subdirectory (tof-stat)

include(GNUInstallDirs)

install(TARGETS get-lens-info jmp-to-boot rapid-changes simple-streamer tof-flip tof-stat tof-discovery
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
