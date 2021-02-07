#ifndef RELATIVE_OBSTACLE_STORING_H
#define RELATIVE_OBSTACLE_STORING_H
#include "obstacle.h"
#include "lidar_message.h"
#include "robot.h"

uint8_t relative_obstacle_storing_lidar_points_relative_to_robot(obstacle_holder_t *holder, lidar_message_t *message, robot_t * robot, float center_offset_degre);
#endif