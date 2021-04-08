#ifndef SPEEDGEN_H
#define SPEEDGEN_H
#include "stdint.h" 
#include "common_types.h"

#define SPEEDGEN_PATH_MAX_LEN   512

typedef struct speedgen_obj
{
    coordinates_t current_path[SPEEDGEN_PATH_MAX_LEN];
}speedgen_obj_t;


speedgen_obj_t speedgen_init_obj();
uint8_t speedgen_import_path(speedgen_obj_t * obj, coordinates_t * path, uint16_t path_len);


#endif