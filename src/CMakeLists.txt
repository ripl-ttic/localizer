add_definitions(
#    -ggdb3 
    -std=gnu99
    )
include_directories(
    ${GLIB2_INCLUDE_DIRS}
    ${GTK2_INCLUDE_DIRS}
    ${LCM_INCLUDE_DIRS})

# localize interface library
set(localize3d_interface_sources
    localize3d_interface.c)

add_library(localize3d_interface ${localize3d_interface_sources})
pods_install_libraries(localize3d_interface)

set(localize3d_motion_sources
    localize3d_motion.c)
add_library(localize3d_motion ${localize3d_motion_sources})

pods_install_libraries(localize3d_motion)

pods_install_headers(localize3d_interface.h
  localize3d_motion.h
  localize3d_messages.h
  localize3dcore.h
  likelihood3d_map.h
  DESTINATION interfaces)

set(localize3d_general
    likelihood3d_map.c
    localize3dcore.c
    localize3d_motion.c
    )

set(localize3d_sources
    localize3d.c 
    ${localize3d_general}
    )

set(localize3d_multi_sources
    localize3d_multi.c 
    ${localize3d_general}
    )

add_executable(hr-localizer ${localize3d_sources})

pods_use_pkg_config_packages(hr-localizer 
    glib-2.0
    gthread-2.0
    lcm 
    bot2-core 
    bot2-lcmgl-client  
    bot2-frames
    bot2-param-client 
    lcmtypes_hr-lcmtypes 
    laser-utils 
    map3d_interface
    carmen-utils
    hr-common)

pods_install_executables(hr-localizer)
