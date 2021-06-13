#ifndef FRAME_H
#define FRAME_H
#include "common_types.h"

#define GLOBAL_FRAME_POS_X 0
#define GLOBAL_FRAME_POS_Y 0
#define GLOBAL_FRAME_REF_X_X 1
#define GLOBAL_FRAME_REF_X_Y 0
#define GLOBAL_FRAME_REF_Y_X 0
#define GLOBAL_FRAME_REF_Y_Y 1

/*
    A frame is defined by its 2 unit vectors 
    and its position according to a global frame.
*/
typedef struct frame
{
    vector_t X;
    vector_t Y;
    coordinates_t pos;
} frame_t;

frame_t get_global_frame();
frame_t set_orthogonal_frame(vector_t *Y, coordinates_t *pos);
coordinates_t point_frame_transformation(frame_t *from, frame_t *to, coordinates_t *point);

#endif