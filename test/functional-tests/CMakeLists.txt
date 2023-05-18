add_subdirectory (jmp-to-boot)
add_subdirectory (simple-streamer)
add_subdirectory (tof-flip)
add_subdirectory (tof-stat)

include(GNUInstallDirs)

install(TARGETS jmp-to-boot simple-streamer tof-stat
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
