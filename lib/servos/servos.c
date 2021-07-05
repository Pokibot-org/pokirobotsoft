#include "servos.h"

LOG_MODULE_REGISTER(servos);

const struct device* pwms_servos;

int servos_init(){

	pwms_servos = device_get_binding(DT_ALIAS_PWMS_SERVOS);
	if (!pwms_servos)
	{
		LOG_ERR("failed to get PWM SERVOS");
		return -1;
	}

	if (pwm_pin_set_usec(pwms_servos, SERVO_1, PERIOD_SERVOS, 0, 0))
	{
		LOG_ERR("PWM pin 1 set fails\n");
		return -1;
	}

	if (pwm_pin_set_usec(pwms_servos, SERVO_2, PERIOD_SERVOS, 0, 0))
	{
		LOG_ERR("PWM pin 2 set fails\n");
		return -1;
	}


	LOG_INF("PWM SERVO initialized.");
	return 0;
}

int servo_set(uint16_t servo, uint16_t val){

	if ((servo != SERVO_1) && (servo != SERVO_2))
		return -1;

	if (val > MAXPULSEWIDTH)
		val = MAXPULSEWIDTH;
	else if (val < MINPULSEWIDTH)
		val = MINPULSEWIDTH;

	if (pwm_pin_set_usec(pwms_servos, servo, PERIOD_SERVOS, val, 0))
	{
		LOG_ERR("PWM servo %d set fails\n", servo);
		return -1;
	}

	return 0;
}