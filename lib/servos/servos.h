#ifndef SERVOS_H
#define SERVOS_H

#include <zephyr.h>
#include <stdint.h>
#include <logging/log.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#define DT_ALIAS_PWMS_SERVOS "PWMS_SERVOS"

typedef enum {
    servo_front_r = 0,
    servo_front_l = 1,
    servo_back_r = 2,
    servo_back_l = 3
} servo_names_t;

typedef struct servo {
    uint8_t channel;
    char * pwm_alias;
    const struct device * pwm_device;
}servo_t;

/* period of servo motor signal ->  20ms (50Hz) */
#define PERIOD_SERVOS (USEC_PER_SEC / 50U)

/* all in micro second */
#define STEP 10   /* PWM pulse step */
#define MINPULSEWIDTH 500  /* Servo 0 degrees */
#define MIDDLEPULSEWIDTH 1500   /* Servo 90 degrees */
#define MAXPULSEWIDTH 2400  /* Servo 180 degrees */

int servos_init();
int servos_set(servo_names_t name, uint16_t val_degree);
void test_servo();

#endif // SERVOS_H