cmake_minimum_required(VERSION 2.6.0)

project(localize3d)

include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB2 REQUIRED glib-2.0)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
pkg_check_modules(LCM REQUIRED lcm)

add_subdirectory(src)