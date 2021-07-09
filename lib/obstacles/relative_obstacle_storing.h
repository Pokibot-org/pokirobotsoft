#ifndef RELATIVE_OBSTACLE_STORING_H
#define RELATIVE_OBSTACLE_STORING_H
#include "obstacle.h"
#include "lidar_message.h"
#include "common_types.h"

uint8_t relative_obstacle_storing_lidar_points_relative_to_robot(obstacle_holder_t *holder, lidar_message_t *message,
                                                                 float robot_angle_rad, coordinates_t robot_pos, 
                                                                 float center_offset_degre);
#endif