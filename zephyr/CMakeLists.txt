cmake_minimum_required(VERSION 3.13.1)

set(DTC_OVERLAY_FILE
    "${DTC_OVERLAY_FILE} project.overlay"
    CACHE STRING "" FORCE
    )

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(pokirobotsoft)

FILE(GLOB app_sources ../src/*.c*)
FILE(GLOB app_sources ../src/ledinfo/*.c*)
target_sources(app PRIVATE ${app_sources})
