#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "ledinfo.h"

// ----------------- DEFINES -------------------------------- 

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif

// ---------------- variables -------------------------------

// ---------------- fun definitions -------------------------
static void error_animation(const struct device * dev);
static void boot_animation(const struct device * dev);
static void running_animation(const struct device * dev);
// ---------------- fun implementation ----------------------

static void (*ledinfo_handler)(const struct device * dev);

static void ledinfo_task(){
    const struct device *dev;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

    boot_animation(dev);

    ledinfo_handler = running_animation;
	while (1) {
        ledinfo_handler(dev);
	}
}


static void error_animation(const struct device * dev){
    static bool led_is_on = true;
    gpio_pin_set(dev, PIN, (int)led_is_on);
    led_is_on = !led_is_on;
    k_msleep(2000);
}

static void boot_animation(const struct device * dev){
    static bool led_is_on = false;
    for (uint8_t i = 0; i < 8; i++)
    {
        gpio_pin_set(dev, PIN, (int)led_is_on);
        led_is_on = !led_is_on;
        k_msleep(50);
    }
}

static void running_animation(const struct device * dev){
    gpio_pin_set(dev, PIN, 1);
    k_msleep(50);
    gpio_pin_set(dev, PIN, 0);
    k_msleep(950);
}


void ledinfo_set_error(){
    ledinfo_handler = error_animation;
}

void ledinfo_set_running(){
    ledinfo_handler = running_animation;
}


K_THREAD_DEFINE(ledinfo_task_name, LEDINFO_STACKSIZE, ledinfo_task, NULL, NULL, NULL,
		LEDINFO_PRIORITY, 0, 0);