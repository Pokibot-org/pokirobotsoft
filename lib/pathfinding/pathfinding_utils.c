#include <math.h>
#include "pathfinding_utils.h"
#ifdef UNIT_TEST
    #include "stdlib.h"
#else
    #include <zephyr>
#endif

uint32_t utils_distance(coordinates_t a, coordinates_t b)
{
    // TODO: only safe is output is at least 33 bit
    return sqrtf(SQUARE((uint32_t)a.x - b.x) + SQUARE((uint32_t)a.y - b.y));
}

uint32_t utils_distance_squared(coordinates_t a, coordinates_t b)
{
    return SQUARE((uint32_t)a.x - b.x) + SQUARE((uint32_t)a.y - b.y);
}

uint32_t utils_distance_summed(coordinates_t a, coordinates_t b)
{
    return abs((uint32_t)a.x - b.x) + abs((uint32_t)a.y - b.y);
}


#ifdef UNIT_TEST
    uint32_t utils_get_rand32(){
        return rand();
    }
#else
    uint32_t utils_get_rand32(){
        return sys_rand32_get();
    }
#endif

/* Normalize with int ... ??? 
void utils_normalize_coordinates(coordinates_t *crd){
    // TODO: check div per 0 ?
    uint32_t max = max(crd->x, crd->y);
    crd->x = crd->x / max;
    crd->y = crd->y / max;
}
*/

uint32_t utils_max(uint32_t a, uint32_t b){
    if(a>=b){
        return a;
    }else{
        return b;
    }
}