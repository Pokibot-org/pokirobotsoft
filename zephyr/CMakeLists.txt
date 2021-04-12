cmake_minimum_required(VERSION 3.13.1)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

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
    ../src/middlewares/obstacle_manager/
    ../src/middlewares/path_manager/
)

FILE(GLOB app_sources 
    ../src/*.c*
    ../src/ledinfo/*.c*
    ../src/drivers/camsense_x1/*.c*
    ../src/middlewares/obstacle_manager/*.c*
    ../src/middlewares/path_manager/*.c*)
target_sources(app PRIVATE ${app_sources})
