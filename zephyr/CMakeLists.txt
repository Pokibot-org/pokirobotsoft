cmake_minimum_required(VERSION 3.13.1)

set(DTC_OVERLAY_FILE
    "${DTC_OVERLAY_FILE} project.overlay"
    CACHE STRING "" FORCE
    )

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(pokirobotsoft)


include_directories(
    ../include
    ../include/nav
    ../include/odometry
)

FILE(GLOB app_sources
    ../src/*.c
    ../src/nav/*.c
    ../src/odometry/*.c
)
target_sources(app PRIVATE ${app_sources})
