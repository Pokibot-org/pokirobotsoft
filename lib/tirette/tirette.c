#include "tirette.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(tirette);

#define TIRETTE_GPIO_LABEL "GPIOA"
#define TIRETTE_GPIO_PIN 9

const static struct device * gpio_port;

uint8_t tirette_init()
{
    gpio_port = device_get_binding(TIRETTE_GPIO_LABEL);
    if (!gpio_port)
    {
        LOG_ERR("Cant get tirette gpio device");
        return 1;
    }

    int err = gpio_pin_configure(gpio_port, TIRETTE_GPIO_PIN, GPIO_INPUT | GPIO_INT_DEBOUNCE| GPIO_PULL_UP);
    if (err)
    {
        LOG_ERR("Cant get tirette gpio device");
        return 1;
    }
    return 0;
}

uint8_t tirette_is_removed()
{
    return gpio_pin_get(gpio_port, TIRETTE_GPIO_PIN);
}

void test_tirette()
{
    tirette_init();
    while (1)
    {
        k_sleep(K_MSEC(100));
        printk("Tirette pin level: %d\n", tirette_is_removed());
    }
    
}