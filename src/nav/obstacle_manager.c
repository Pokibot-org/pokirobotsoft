#include <zephyr.h>
#include <logging/log.h>
#include <math.h>
#include "lidar.h"
#include "relative_obstacle_storing.h"
#include "nav/obstacle_manager.h"
#include "robot_config.h"
#include "odometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.78539816339744830962f
#endif

LOG_MODULE_REGISTER(obstacle_manager, 2);

K_MSGQ_DEFINE(obstacle_manager_msgq, sizeof(obstacle_manager_message_t), 10, 1);

// PRIVATE VAR
static obstacle_manager_t obs_man_obj = {0};
K_SEM_DEFINE(obsacle_holder_lock, 1, 1);
// PRIVATE DEF
#define CAMSENSE_CENTER_OFFSET_DEG 16.0f
#define LIDAR_COUNTER_CLOCKWISE
#define LIDAR_DETECTION_DISTANCE_MM 250
#define LIDAR_DETECTION_ANGLE 100
// FUNC

void obstacle_manager_send_message(const obstacle_manager_message_t *msg)
{
    while (k_msgq_put(&obstacle_manager_msgq, msg, K_NO_WAIT))
    {
        LOG_ERR("Queue full, purging");
        k_msgq_purge(&obstacle_manager_msgq);
    }
}

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj)
{
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    memcpy(obj, &obs_man_obj.obs_holder, sizeof(obs_man_obj.obs_holder));
    k_sem_give(&obsacle_holder_lock);
    return 0;
}

uint8_t obstacle_manager_is_there_an_obstacle()
{
    return obs_man_obj.obstacle_detected;
}

void on_rotation_lidar_callback()
{
    obstacle_manager_message_t msg = {
        .type = obstacle_manager_message_type_event,
        .event = obstacle_manager_message_event_lidar_did_rotation};
    obstacle_manager_send_message(&msg);
}

uint8_t process_point(obstacle_manager_t *obj, distance_t point_distance, float point_angle)
{
    uint8_t return_code = 0;
    obstacle_t new_obstacle = {
        .type = obstacle_type_circle,
        .data.circle.radius = 0 // FIXME: remove the magic number
    };

    pos_t actual_robot_pos = robot_get_pos();

    // LOG_INF("IN PROCEES POINT: angle: %f, distance: %d", point_angle, point_distance);

    if (((point_distance < ROBOT_MAX_RADIUS_MM) && (ABS(point_angle) > LIDAR_DETECTION_ANGLE / 2)) ||
        (point_distance < ROBOT_MIN_RADIUS_MM)) // in robot do nothing
    {
        // LOG_INF("Point in robot");
        return 0;
    }

    // if it is a near obstacle change return code
    if ((point_distance < ROBOT_MAX_RADIUS_MM + LIDAR_DETECTION_DISTANCE_MM) && (ABS(point_angle) < LIDAR_DETECTION_ANGLE / 2))
    {
        LOG_DBG("Obstacle detected | angle: %.3hi, distance: %.5hu", (int16_t)(point_angle), point_distance);
        return_code = 1;
    }

    float point_angle_absolute = ((point_angle * (M_PI / 180.0f)) + actual_robot_pos.a_rad);
    if (point_angle_absolute < -M_PI_2)
    {
        point_angle_absolute += M_PI;
    }
    else if (point_angle_absolute > M_PI_2)
    {
        point_angle_absolute -= M_PI;
    }

    new_obstacle.data.circle.coordinates.x = actual_robot_pos.x +
                                             sinf(point_angle_absolute) * point_distance;
    new_obstacle.data.circle.coordinates.y = actual_robot_pos.y +
                                             cosf(point_angle_absolute) * point_distance;

    if (new_obstacle.data.circle.coordinates.x < 0 ||
        new_obstacle.data.circle.coordinates.x > 3000 ||
        new_obstacle.data.circle.coordinates.y < 0 ||
        new_obstacle.data.circle.coordinates.y > 2000)
    {
        return 2;
    }
    // Uncomment the following lines if you want to use tools/lidar_point_visualiser.py
    // printk("<%hd:%hd>\n", new_obstacle.data.circle.coordinates.x, new_obstacle.data.circle.coordinates.y);
    obstacle_holder_push_circular_buffer_mode(&obj->obs_holder, &new_obstacle);

    return return_code;
}

uint8_t process_lidar_message(obstacle_manager_t *obj, const lidar_message_t *message)
{
    float step = 0.0f;
    uint8_t in_obstacle_detected = 0;
    if (message->end_angle > message->start_angle)
    {
        step = (message->end_angle - message->start_angle) / 8.0f;
    }
    else
    {
        step = (message->end_angle - (message->start_angle - 360.0f)) / 8.0f;
    }

    for (int i = 0; i < LIDAR_MESSAGE_NUMBER_OF_POINT; i++) // for each of the 8 samples
    {
        if (message->points[i].quality != 0) // Filter some noisy data
        {
#ifdef LIDAR_COUNTER_CLOCKWISE
            float point_angle = 360.0f - ((message->start_angle + step * i) + (CAMSENSE_CENTER_OFFSET_DEG + 180.0f));
#else
            float point_angle = (message->start_angle + step * i) + (CAMSENSE_CENTER_OFFSET_DEG + 180.0f);
#endif

            uint8_t err_code = process_point(&obs_man_obj, message->points[i].distance, point_angle);
            if (err_code == 1) // 0 ok, 1 in front of robot, 2 outside table
            {
                in_obstacle_detected = 1;
            }
        }
    }

    obj->obstacle_detected = in_obstacle_detected;
    if (obj->obstacle_detected)
    {
        LOG_INF("Obstacle detected in front of the robot");
    }
    return 0;
}

static void obstacle_manager_task()
{
    camsense_x1_init();
    camsense_x1_add_on_rotation_clbk(on_rotation_lidar_callback);
    int err;
    obstacle_manager_message_t msg;
    while (true)
    {
        k_msgq_get(&obstacle_manager_msgq, &msg, K_FOREVER);
        LOG_DBG("Recived message");
        if (msg.type == obstacle_manager_message_type_event)
        {
            LOG_DBG("Recived event");
            if (msg.event == obstacle_manager_message_event_lidar_did_rotation)
            {
                LOG_DBG("Recived lidar_did_rotation event");
                lidar_message_t lidar_message;
                k_sem_take(&obsacle_holder_lock, K_FOREVER);
                while (!camsense_x1_read_sensor_non_blocking(&lidar_message))
                {
                    err = process_lidar_message(&obs_man_obj, &lidar_message);
                    if (err)
                    {
                        LOG_ERR("obstacle_manager_task error when calling process_lidar_message %d", err);
                    }
                }
                k_sem_give(&obsacle_holder_lock);
            }
        }
    }
}

#if CONFIG_OBSTACLE_MANAGER_THREAD_ENABLED
K_THREAD_DEFINE(obstacle_manager_task_name,
                CONFIG_OBSTACLE_MANAGER_THREAD_STACK,
                obstacle_manager_task,
                NULL, NULL, NULL,
                CONFIG_OBSTACLE_MANAGER_THREAD_PRIORITY, 0, 0);
#endif
