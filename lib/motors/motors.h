#ifndef MOTORS_H
#define MOTORS_H

#include <zephyr.h>
#include <logging/log.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#define DT_ALIAS_PWMS_MOTORS "PWMS_MOTORS"
#define MOTOR_L 1
#define MOTOR_R 2

#define FREQUENCY 20000U
#define PERIOD (USEC_PER_SEC / FREQUENCY)

#define PWM_MAX 50
#define PWM_MIN 0

int motors_init();
int motor_set(uint16_t motor, int16_t val);

#endif // MOTORS_H