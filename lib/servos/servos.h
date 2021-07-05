#ifndef SERVOS_H
#define SERVOS_H

#include <zephyr.h>
#include <logging/log.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#define DT_ALIAS_PWMS_SERVOS "PWMS_SERVOS"
#define SERVO_1 1
#define SERVO_2 2

/* period of servo motor signal ->  20ms (50Hz) */
#define PERIOD_SERVOS (USEC_PER_SEC / 50U)

/* all in micro second */
#define STEP 10   /* PWM pulse step */
#define MINPULSEWIDTH 500  /* Servo 0 degrees */
#define MIDDLEPULSEWIDTH 1500   /* Servo 90 degrees */
#define MAXPULSEWIDTH 2400  /* Servo 180 degrees */

int servos_init();
int servo_set(uint16_t servo, uint16_t val);

#endif // SERVOS_H