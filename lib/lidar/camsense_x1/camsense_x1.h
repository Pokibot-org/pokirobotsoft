#ifndef CAMSENSE_X1_H
#define CAMSENSE_X1_H
#include <lidar_message.h>

typedef void (*camsense_x1_on_rotation_clbk)();  // void * == user data

int camsense_x1_init();
float camsense_x1_get_sensor_speed();
int camsense_x1_read_sensor_blocking(lidar_message_t *message);
int camsense_x1_read_sensor_non_blocking(lidar_message_t *message);
void camsense_x1_add_on_rotation_clbk(camsense_x1_on_rotation_clbk fun);

#endif