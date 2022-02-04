#include "zephyr.h"
#include "wall_detector.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(wall_detector);

#define WALL_DETECTOR_GPIO_LABEL "GPIOC"
const static int wd_pins[] = {5,6,8,9};
#define NUMBER_OF_WD ARRAY_SIZE(wd_pins)
const static struct device * gpio_port;

uint8_t wall_detector_init()
{
    gpio_port = device_get_binding(WALL_DETECTOR_GPIO_LABEL);
    if (!gpio_port)
    {
        LOG_ERR("Cant get wall_detector gpio device");
        return 1;
    }

    for (size_t i = 0; i < NUMBER_OF_WD; i++)
    {
        int err = gpio_pin_configure(gpio_port, wd_pins[i], GPIO_INPUT | GPIO_INT_DEBOUNCE | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
        if (err)
        {
            LOG_ERR("Cant config pin %d", i);
            return 1;
        }
    }
    
    return 0;
}

uint8_t wd_get_collision_bitmask()
{
    if (gpio_port == NULL)
    {
        LOG_ERR("Wall detector not initialised");
        return 255;
    }

    uint8_t bitmask = 0;
    for (size_t i = 0; i < NUMBER_OF_WD; i++)
    {
        bitmask = bitmask << 1;
        bitmask |= gpio_pin_get(gpio_port, wd_pins[i]) & 0x01;
    }
    
    return bitmask;
}


uint8_t wd_front_is_touching()
{
    uint8_t bitmask = wd_get_collision_bitmask();
    return (bitmask & 0xC) == 0xC;
}

uint8_t wd_back_is_touching()
{
    uint8_t bitmask = wd_get_collision_bitmask();
    return (bitmask & 0x3) == 0x3;
}

uint8_t wd_is_activated(wd_name_t name)
{
    if (gpio_port == NULL)
    {
        LOG_ERR("Wall detector not initialised");
        return 255;
    }
    return gpio_pin_get(gpio_port, wd_pins[(uint8_t)name]);
}

void test_wall_detector()
{
    wall_detector_init();
    while (1)
    {
        k_sleep(K_MSEC(100));
        printk("Bitmast wd : %d\n", wd_get_collision_bitmask());
    }
    
}