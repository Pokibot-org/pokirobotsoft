cmake_minimum_required(VERSION 3.13.1)

set(DTC_OVERLAY_FILE
    "${DTC_OVERLAY_FILE} project.overlay"
    CACHE STRING "" FORCE
    )

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(pokirobotsoft)


include_directories(
    ../src/
    ../src/ledinfo
    ../src/drivers/camsense_x1/
)

FILE(GLOB app_sources 
    ../src/*.c*
    ../src/ledinfo/*.c*
    ../src/drivers/camsense_x1/*.c*)
target_sources(app PRIVATE ${app_sources})
