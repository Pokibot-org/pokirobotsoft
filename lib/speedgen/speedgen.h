#ifndef SPEEDGEN_H
#define SPEEDGEN_H
#include "common_types.h"
#include "robot.h"
#include "vector.h"
#include "frame.h"

#define SPEEDGEN_PATH_MAX_LEN   512
#define LOOKAHEAD_DIST  1   //Robot follow a point a this distance in Meters
#define REF_V_MAX 1 //m/s
#define REF_A_MAX 3 //m/s^2

typedef struct section
{
    coordinates_t begin_pos;    //Beginnig of the Section
    vector_t end_pos;           //Vector pointing to the end of the Section
    uint16_t current_path_index;        //current section "begin_pos" Coordinates index position in the "current_path"
    long_distance_t begin_distance;     //Distance along the Path of the Beggining of the Section from the first Point of the Path
    distance_t section_distance;        //Distance of the Section
} section_t;

typedef struct speed_profile
{
    uint16_t S1;    //Distance of the End of the 1st Phase
    uint16_t S2;    //Distance of the End of the 2nd Phase  
}speed_profile_t;

typedef struct speedgen_obj
{
    coordinates_t current_path[SPEEDGEN_PATH_MAX_LEN];
    uint16_t path_len;
    uint16_t path_distance;     //Path Distance in m
    robot_t *robot;
    section_t *current_section;  //A section of a path is composed of two consecutive Waypoints
    speed_profile_t *speed_profile;
}speedgen_obj_t;

void speedgen_init_obj(speedgen_obj_t * obj);
void speedgen_current_path_init(coordinates_t *current_path);
void speedgen_current_section_init(section_t *current_section);
void speedgen_speed_profile_init(speed_profile_t *speed_profile);
uint8_t speedgen_import_path(speedgen_obj_t * obj, coordinates_t * path, uint16_t path_len);
uint8_t speedgen_set_speed_profile(speedgen_obj_t *obj);
vector_t speedgen_find_path_point(speedgen_obj_t * obj);
coordinates_t speedgen_compute_goal_point(speedgen_obj_t * obj, vector_t * path_point);
section_t speedgen_get_next_section(speedgen_obj_t * obj);
coordinates_t speedgen_goal_point_frame_transformation(coordinates_t * goal_point, speedgen_obj_t * obj);
float speedgen_compute_curvature(coordinates_t * goal_point, speedgen_obj_t * obj);
uint16_t speedgen_get_longi_speed(speedgen_obj_t * obj, float curvature);
uint16_t speedgen_get_angular_speed(uint16_t longi_speed, float curvature);

#endif