#include "tirette.h"
#include "logging/log.h"

LOG_MODULE_REGISTER(tirette);

#define TIRETTE_GPIO_LABEL "GPIOA"
#define TIRETTE_GPIO_PIN 9

const static struct device * gpio_port;
// K_SEM_DEFINE(tirette_init_lock, 1, 1);
K_MUTEX_DEFINE(tirette_mutex);

uint8_t tirette_init()
{
    if(!k_mutex_lock(&tirette_mutex, K_NO_WAIT))
    {
        LOG_WRN("Tirette already init");
        // return 0; // FIXME: Find a way to init tirette in all threads 
    }

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
    return !gpio_pin_get(gpio_port, TIRETTE_GPIO_PIN);
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
