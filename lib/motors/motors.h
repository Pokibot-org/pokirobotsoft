#ifndef MOTORS_H
#define MOTORS_H

#include <zephyr.h>
#include <logging/log.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>

#define DT_ALIAS_PWMS_MOTORS "PWMS_MOTORS"
#define MOTOR_L 1
#define MOTOR_R 2

#define WAY_FORWARD 	true
#define WAY_BACKWARD	false

#define DIR_L_PIN_PORT	"GPIOC"
#define DIR_L_PIN_NUM	0
#define DIR_L_PIN_FORWARD true // Edit here if motor turn the wrong way
#define DIR_R_PIN_PORT	"GPIOC"
#define DIR_R_PIN_NUM	1
#define DIR_R_PIN_FORWARD false // Edit here if motor turn the wrong way

#define FREQUENCY_MOTORS 20000U
#define PERIOD_MOTORS (USEC_PER_SEC / FREQUENCY_MOTORS)

#define PWM_MAX 50
#define PWM_MIN -PWM_MAX

int motors_init();
int motor_set(uint16_t motor, int16_t val);

#endif // MOTORS_H
