#include <math.h>
#include <stdlib.h>

#include "frame.h"
#include "vector.h"

#ifndef UNIT_TEST
#include <zephyr.h>
#endif

//Function returning the Global frame according to the 
//right low corner of the table. 
frame_t get_global_frame()
{
    frame_t global_frame;

    global_frame.pos.x = GLOBAL_FRAME_POS_X;
    global_frame.pos.y = GLOBAL_FRAME_POS_Y;
    global_frame.X.x = GLOBAL_FRAME_REF_X_X;
    global_frame.X.y = GLOBAL_FRAME_REF_X_Y;
    global_frame.Y.x = GLOBAL_FRAME_REF_Y_X;
    global_frame.Y.y = GLOBAL_FRAME_REF_Y_Y;

    return global_frame;
};

//Returning an orthogonal frame from the Y vector ans its position
//in the global frame
frame_t set_orthogonal_frame(vector_t *Y, coordinates_t *pos)
{
    frame_t frame;

    frame.pos = *pos;
    frame.Y = *Y;
    //Computing the orthogonal X unit vector 
    frame.X.y = -sqrtf(1/((frame.Y.y/frame.Y.x)+1));
    frame.X.x = -frame.X.y*frame.Y.y/frame.Y.x;

    return frame;
};

//Function converting point coordinates to another frame
coordinates_t point_frame_transformation(frame_t *from, frame_t *to, coordinates_t *point)
{
    coordinates_t new_point;

    //Translation of the frame
    vector_t F_T = utils_vector_from_points(&from->pos,&to->pos);
    vector_t F_P = utils_vector_from_points(&from->pos,point);

    point_t X_bis = F_P.x - F_T.x;
    point_t Y_bis = F_P.y - F_T.y;

    //Rotation of the frame
    float theta = utils_get_vector_angle(&from->Y,&to->Y);
    new_point.x = cos(theta)*X_bis - sin(theta)*Y_bis;
    new_point.y = sin(theta)*X_bis + cos(theta)*Y_bis;

    return new_point;
};