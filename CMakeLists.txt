cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME localizer)

include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB2 REQUIRED glib-2.0)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(LCM REQUIRED lcm)

pods_install_pkg_config_file(lcmtypes_${POD_NAME}
    LIBS -l${LCMTYPES_LIBS}
    REQUIRES lcm
    VERSION 0.0.1)

add_subdirectory(src)
