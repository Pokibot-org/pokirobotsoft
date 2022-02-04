#ifndef SPEEDGEN_H
#define SPEEDGEN_H
#include "stdint.h" 
#include "common_types.h"
#include "vector.h"
#include "frame.h"

#define SPEEDGEN_PATH_MAX_LEN   512
#define LOOKAHEAD_DIST  1   //Robot follow a point a this distance in Meters

typedef struct section
{
    coordinates_t begin_pos;    //Beginnig of the Section
    vector_t end_pos;           //Vector pointing to the end of the Section
    uint16_t current_path_index;        //current section "begin_pos" Coordinates index position in the "current_path"
    long_distance_t begin_distance;     //Distance along the Path of the Beggining of the Section from the first Point of the Path
    distance_t section_distance;        //Distance of the Section
} section_t;

typedef struct speedgen_obj
{
    coordinates_t current_path[SPEEDGEN_PATH_MAX_LEN];
    uint16_t path_len;
    section_t *current_section;  //A section of a path is composed of two consecutive Waypoints

}speedgen_obj_t;

speedgen_obj_t speedgen_init_obj();
uint8_t speedgen_import_path(speedgen_obj_t * obj, coordinates_t * path, uint16_t path_len);
vector_t find_path_point(speedgen_obj_t * obj);
coordinates_t compute_goal_point(speedgen_obj_t * obj, vector_t * path_point);
section_t get_next_section(speedgen_obj_t *obj);

#endif