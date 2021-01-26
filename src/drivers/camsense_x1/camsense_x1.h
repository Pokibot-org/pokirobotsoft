#ifndef CAMSENSE_X1_H
#define CAMSENSE_X1_H

#define CAMSENSE_X1_NUMBER_OF_POINT 8


typedef struct {
    uint16_t distance;
    uint8_t quality;
} lidar_point_t;

typedef struct {
    float start_angle;
    float end_angle;
    lidar_point_t points[CAMSENSE_X1_NUMBER_OF_POINT];
} lidar_message_t;


int camsense_x1_init();
float camsense_x1_get_sensor_speed();
void camsense_x1_read_sensor(lidar_message_t *message);

#endif