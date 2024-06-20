# create library
nuttx_add_subdirectory(core)
nuttx_add_subdirectory(inc)
nuttx_add_subdirectory(src)

add_library(uMsgLib STATIC
    umsg.c
    # generated files
    battery.c
    control.c
    example.c
    sensors.c
    test.c
    unused.c
)
# select port
if(NOT DEFINED UMSG_PORT)
    message(FATAL_ERROR "UMSG_PORT is not defined")
endif()
        target_sources(uMsgLib PRIVATE
                port_posix.c
        )

# add includes
# target_include_directories(uMsgLib PUBLIC inc core/inc)
target_link_options(uMsgLib PRIVATE -ffunction-sections -fdata-sections)
# linker options to allow dead code elimination
target_link_options(uMsgLib PRIVATE -Wl,--gc-sections)