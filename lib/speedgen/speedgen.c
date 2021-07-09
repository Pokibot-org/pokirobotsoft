#include <math.h>

#include "speedgen.h"
#include "string.h"

speedgen_obj_t speedgen_init_obj()
{
    speedgen_obj_t new_obj = {0};
    return new_obj;
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

//  Function Searching for the closest point on the path according to the robot position
//  Return a Vector pointing to this point from the beginning of the Section
vector_t find_path_point(speedgen_obj_t * obj)
{
    vector_t vect;

    //FIXME: REPLACE THE NULL
    vector_t robot_v = utils_vector_from_points(&obj->current_section->begin_pos, NULL);
    uint32_t robot_dist = utils_vector_norm(&robot_v);
    float theta = utils_get_vector_angle(&obj->current_section->end_pos,&robot_v);
    //Projection of robot position on section
    uint32_t proj_dist = robot_dist*cos(theta);
    vector_t unit_vector = utils_get_unit_vector(&obj->current_section->end_pos);
    //Getting Point Vector
    int32_t x = proj_dist*unit_vector.x;
    int32_t y = proj_dist*unit_vector.y;
    vect.x = x;
    vect.y = y;

    return vect;
}

//  Function Computing the Goal Point from a Section and a path_point Vector
//  Returning the coordinates of the Goal Point in Global Frame
coordinates_t compute_goal_point(speedgen_obj_t * obj, vector_t * path_point){

    uint32_t goal_norm = utils_vector_norm(path_point)+LOOKAHEAD_DIST;
    
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
            *obj->current_section = get_next_section(obj);
        };

        goal_norm = section_goal_norm;
    };

    vector_t truc = utils_get_unit_vector(&obj->current_section->end_pos);
    vector_t bidule = utils_form_vector(&truc, &goal_norm);
    return utils_coordinates_from_vector(&bidule, &obj->current_section->begin_pos);
};

//Function Returning the next section parameters
section_t get_next_section(speedgen_obj_t *obj)
{
    section_t next_section;

    //Update the index 
    next_section.current_path_index = obj->current_section->current_path_index+1;
    //get the current section end point
    next_section.begin_pos = obj->current_path[next_section.current_path_index];
    //get the end position
    coordinates_t section_end = obj->current_path[next_section.current_path_index+1];
    //convert it to a vector pointing to it
    next_section.end_pos = utils_vector_from_points(&next_section.begin_pos,&section_end);
    //Complete the last parameters
    next_section.begin_distance = obj->current_section->begin_distance + obj->current_section->section_distance;
    next_section.section_distance = utils_vector_norm(&next_section.end_pos);

    return next_section;
};