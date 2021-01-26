#include <zephyr.h>
#include <logging/log.h>
#include "camsense_x1.h"


LOG_MODULE_REGISTER(main);

void main(void)
{
    camsense_x1_init();
}