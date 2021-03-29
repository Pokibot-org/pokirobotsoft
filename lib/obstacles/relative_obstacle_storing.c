#ifdef UNIT_TEST
#include <math.h>
#else
#include <math.h>
#include <zephyr.h>
#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#endif
#include "relative_obstacle_storing.h"

uint8_t relative_obstacle_storing_lidar_points_relative_to_robot(obstacle_holder_t *holder, lidar_message_t *message, robot_t * robot, float center_offset_degre)
{
    float step = 0.0f;
    obstacle_t new_obstacle = {
        .type = obstacle_type_circle,
        .data.circle.diameter = 8 // FIXME: remove the magic number
    };

    if (message->end_angle > message->start_angle)
    {
        step = (message->end_angle - message->start_angle) / 8;
    }
    else
    {
        step = (message->end_angle - (message->start_angle - 360.0f)) / 8;
    }

    for (int i = 0; i < LIDAR_MESSAGE_NUMBER_OF_POINT; i++) // for each of the 8 samples
    {
        float point_angle = (message->start_angle + step * i) + (center_offset_degre + 180);
        float point_angle_absolute = ((point_angle*M_PI/180) + robot->angle_rad);
        if (point_angle_absolute < -M_PI_2)
        {
            point_angle_absolute += M_PI;
        }
        else if (point_angle_absolute > M_PI_2)
        {
            point_angle_absolute -= M_PI;
        }
        

        if (message->points[i].quality != 0) // Filter some noisy data
        {
            // if (message->points[i].distance <= robot->radius_mm){
            //     continue;
            // }


            new_obstacle.data.circle.coordinates.x = robot->position.x + sinf(point_angle_absolute) * message->points[i].distance;
            new_obstacle.data.circle.coordinates.y = robot->position.y + cosf(point_angle_absolute) * message->points[i].distance;

            printk("<%hd:%hd>\n", new_obstacle.data.circle.coordinates.x, new_obstacle.data.circle.coordinates.y);
            obstacle_holder_push_circular_buffer_mode(holder, &new_obstacle);
        }
    }
    return 0;
}
