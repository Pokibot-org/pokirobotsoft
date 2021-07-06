#ifndef LIDAR_MESSAGE_H
#define LIDAR_MESSAGE_H

#include "stdint.h"

#define LIDAR_MESSAGE_NUMBER_OF_POINT 8

typedef struct {
    uint16_t distance;
    uint8_t quality;
} lidar_point_t;

typedef struct {
    float start_angle;
    float end_angle;
    lidar_point_t points[LIDAR_MESSAGE_NUMBER_OF_POINT];
} lidar_message_t;

#endif
