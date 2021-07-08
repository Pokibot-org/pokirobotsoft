#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include "flag/flag.h"

LOG_MODULE_REGISTER(flag);

static const struct device * flag_port = {0};

uint8_t flag_init()
{
    flag_port = device_get_binding(FLAG_PORT_LABEL);
    if (!flag_port)
    {
        LOG_ERR("Cant get flag gpio device");
        return 1;
    }

    int err = gpio_pin_configure(flag_port, FLAG_PIN, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    if (err)
    {
        LOG_ERR("Cant get tirette gpio device");
        return 1;
    }
    return 0;
}

uint8_t raise_flag()
{
    if (flag_port == NULL)
    {
        LOG_ERR("Flag not initialized");
        return 1;
    }

    int err = gpio_pin_set(flag_port, FLAG_PIN, 1); 
    if (err)
    {
        LOG_ERR("Error when setting pin level");
        return 1;
    }

    return 0;
}
