#ifndef CAMSENSE_X1_H
#define CAMSENSE_X1_H
#include "lidar_messages.h"

int camsense_x1_init();
float camsense_x1_get_sensor_speed();
void camsense_x1_read_sensor(lidar_message_t *message);

#endif