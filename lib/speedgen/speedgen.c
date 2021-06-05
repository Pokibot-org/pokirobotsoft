#include <math.h>

#include "speedgen.h"
#include "string.h"

void speedgen_init_obj(speedgen_obj_t *new_obj)
{
    new_obj->path_len = 0;
    new_obj->path_distance = 0;
    new_obj->robot = NULL;
    speedgen_speed_profile_init(new_obj->speed_profile);
    speedgen_current_path_init(new_obj->current_path);
    speedgen_current_section_init(new_obj->current_section);
}

void speedgen_current_path_init(coordinates_t *current_path)
{
    int idx;
    for (idx=0; idx<SPEEDGEN_PATH_MAX_LEN; idx++)
    {
        current_path[idx].x=0;
        current_path[idx].y=0;
    }
}

void speedgen_current_section_init(section_t *current_section)
{
    current_section->begin_distance = 0;
    current_section->begin_pos.x = 0;
    current_section->begin_pos.y = 0;
    current_section->current_path_index = 0;
    current_section->end_pos.x = 0;
    current_section->end_pos.x = 0;
    current_section->section_distance = 0; 
}

void speedgen_speed_profile_init(speed_profile_t *speed_profile)
{
    speed_profile->S1 = 0;
    speed_profile->S2 = 0;
}

uint8_t speedgen_import_path(speedgen_obj_t * obj, coordinates_t * path, uint16_t path_len)
{
    if (path_len >= SPEEDGEN_PATH_MAX_LEN)
    {
        return 1;
    }
    memcpy(obj->current_path, path, path_len * sizeof(coordinates_t));
    return 0;
}

uint8_t speedgen_set_speed_profile(speedgen_obj_t *obj)
{
    if (obj->path_len==0)
    {
        return 1;
    }
    if (obj->robot==NULL)
    {
        return 2;
    }
    obj->speed_profile->S1 = (SQUARE(REF_V_MAX))/(2*REF_A_MAX);
    if (obj->speed_profile->S1 > obj->path_distance/2){
        obj->speed_profile->S2 = obj->path_distance-obj->speed_profile->S1;
    }
    else obj->speed_profile->S2 = 0;

    return 0;
};

//  Function Searching for the closest point on the path according to the robot position
//  Return a Vector pointing to this point from the beginning of the Section
vector_t speedgen_find_path_point(speedgen_obj_t * obj)
{
    vector_t vect;

    vector_t robot_v = vector_vector_from_points(&obj->current_section->begin_pos,&obj->robot->position);
    uint32_t robot_dist = VECTOR_NORM(robot_v);
    float theta = vector_get_vector_angle(&obj->current_section->end_pos,&robot_v);
    //Projection of robot position on section
    uint32_t proj_dist = robot_dist*cos(theta);
    vector_t unit_vector = vector_get_unit_vector(&obj->current_section->end_pos);
    //Getting Point Vector
    int32_t x = proj_dist*unit_vector.x;
    int32_t y = proj_dist*unit_vector.y;
    vect.x = x;
    vect.y = y;

    return vect;
}

//  Function Computing the Goal Point from a Section and a path_point Vector
//  Returning the coordinates of the Goal Point in Global Frame
coordinates_t speedgen_compute_goal_point(speedgen_obj_t * obj, vector_t * path_point){

    uint32_t goal_norm = VECTOR_NORM((*path_point))+LOOKAHEAD_DIST;
    
    if (goal_norm > obj->current_section->section_distance)
    {
        uint32_t section_goal_norm = goal_norm;
        /* 
            While the section_goal_norm of the new section is greater than the
            new section norm, we pass to the next section
        */
        while (section_goal_norm > obj->current_section->section_distance)
        {
            //If we are at the final section, return the final point
            if (obj->current_section->current_path_index == obj->path_len-1)
            {
                return obj->current_path[obj->path_len-1];
            }
            section_goal_norm = section_goal_norm - obj->current_section->section_distance;
            //Updating the current section by the new one
            *obj->current_section = speedgen_get_next_section(obj);
        };

        goal_norm = section_goal_norm;
    };
    //Get Section direction
    vector_t unit_vect = vector_get_unit_vector(&obj->current_section->end_pos);
    //Apply the goal norm
    vector_t goal_vect = vector_form_vector(&unit_vect,&goal_norm);

    return vector_coordinates_from_vector(
                &goal_vect,
                &obj->current_section->begin_pos);
};

//Function Returning the next section parameters
section_t speedgen_get_next_section(speedgen_obj_t *obj)
{
    section_t next_section;

    //Update the index 
    next_section.current_path_index = obj->current_section->current_path_index+1;
    //get the current section end point
    next_section.begin_pos = obj->current_path[next_section.current_path_index];
    //get the end position
    coordinates_t section_end = obj->current_path[next_section.current_path_index+1];
    //convert it to a vector pointing to it
    next_section.end_pos = vector_vector_from_points(&next_section.begin_pos,&section_end);
    //Complete the last parameters
    next_section.begin_distance = obj->current_section->begin_distance + obj->current_section->section_distance;
    next_section.section_distance = VECTOR_NORM(next_section.end_pos);

    return next_section;
};

//Function Changing the reference frame of the Goal point from Global -> Robot
coordinates_t speedgen_goal_point_frame_transformation(coordinates_t * goal_point, speedgen_obj_t * obj)
{
    coordinates_t goal_point_robot_frame;

    //Create the robot frame
    vector_t robot_frame_Y = vector_get_vector_from_angle(&obj->robot->angle_rad);
    frame_t robot_frame = frame_set_orthogonal_frame(&robot_frame_Y,&obj->robot->position);
    //Gobal frame
    frame_t global_frame = frame_get_global_frame();
    //Goal point Global Frame -> Goal point Robot Frame
    goal_point_robot_frame = frame_point_frame_transformation(&global_frame,&robot_frame,goal_point);

    return goal_point_robot_frame;
};

//Function computing the curvature of the path to the goal point
float speedgen_compute_curvature(coordinates_t * goal_point, speedgen_obj_t * obj)
{
    float curvature;

    //Get robot distance from the goal
    float robot_distance = sqrtf(SQUARE(goal_point->x) + SQUARE(goal_point->y));

    //Compute Curvature
    curvature = (2*ABS(goal_point->x))/(SQUARE(robot_distance));

    return curvature;
};

//Function computing the desired longitudinal Speed
uint16_t speedgen_get_longi_speed(speedgen_obj_t * obj, float curvature)
{
    return 0;
};