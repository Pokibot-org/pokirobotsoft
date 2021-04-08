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
