#include "motors.h"

LOG_MODULE_REGISTER(motors);

const struct device* pwms_motors;

int motors_init(){

	pwms_motors = device_get_binding(DT_ALIAS_PWMS_MOTORS);
	if (!pwms_motors)
	{
		LOG_ERR("failed to get PWM MOTORS");
		while (1)
		{
			k_sleep(K_MSEC(1000));
			LOG_INF("fail device_get PWM MOTORS");
		}
	}

	if (pwm_pin_set_usec(pwms_motors, MOTOR_L, PERIOD, 0, 0))
	{
		LOG_ERR("PWM pin 1 set fails\n");
		while (1)
		{
			k_sleep(K_MSEC(1000));
			LOG_INF("PWM pin 1 set fails");
		}
	}

	if (pwm_pin_set_usec(pwms_motors, MOTOR_R, PERIOD, 0, 0))
	{
		LOG_ERR("PWM pin 2 set fails\n");
		while (1)
		{
			k_sleep(K_MSEC(1000));
			LOG_INF("PWM pin 2 set fails");
		}
	}

	LOG_INF("PWM MOTORS initialized.");
	return 0;
}

int motor_set(uint16_t motor, int16_t val){

	if ((motor != MOTOR_L) && (motor != MOTOR_R))
		return 1;

	if (val > PWM_MAX)
		val = PWM_MAX;
	else if (val < PWM_MIN)
		val = PWM_MIN;


	if (pwm_pin_set_usec(pwms_motors, motor, PERIOD, val, 0))
	{
		LOG_ERR("PWM motor %d set fails\n", motor);
		while (1)
		{
			k_sleep(K_MSEC(1000));
			LOG_ERR("PWM motor %d set fails\n", motor);
		}
	}

	return 0;
}