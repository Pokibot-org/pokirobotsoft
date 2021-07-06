#include "motors.h"

LOG_MODULE_REGISTER(motors);

static const struct device* pwms_motors;
static const struct device *dir_l;
static const struct device *dir_r;

int motor_set_dir(uint16_t motor, bool way);

int motors_init(){

	pwms_motors = device_get_binding(DT_ALIAS_PWMS_MOTORS);
	if (!pwms_motors)
	{
		LOG_ERR("failed to get PWM MOTORS");
		return -1;
	}

	if (pwm_pin_set_usec(pwms_motors, MOTOR_L, PERIOD_MOTORS, 0, 0))
	{
		LOG_ERR("PWM pin 1 set fails\n");
		return -1;
	}

	if (pwm_pin_set_usec(pwms_motors, MOTOR_R, PERIOD_MOTORS, 0, 0))
	{
		LOG_ERR("PWM pin 2 set fails\n");
		return -1;
	}

	dir_l = device_get_binding(DIR_L_PIN_PORT);
	gpio_pin_configure(dir_l, DIR_L_PIN_NUM, GPIO_OUTPUT_ACTIVE);

	dir_r = device_get_binding(DIR_R_PIN_PORT);
	gpio_pin_configure(dir_r, DIR_R_PIN_NUM, GPIO_OUTPUT_ACTIVE);

	LOG_INF("PWM MOTORS initialized.");
	return 0;
}

int motor_set(uint16_t motor, int16_t val){

	if ((motor != MOTOR_L) && (motor != MOTOR_R))
		return -1;

	if (val > PWM_MAX)
		val = PWM_MAX;
	else if (val < PWM_MIN)
		val = PWM_MIN;

	if (val < 0){
		val = -val;
		motor_set_dir(motor, WAY_BACKWARD);
	} else
		motor_set_dir(motor, WAY_FORWARD);


	if (pwm_pin_set_usec(pwms_motors, motor, PERIOD_MOTORS, val, 0))
	{
		LOG_ERR("PWM motor %d set fails\n", motor);
		return -1;
	}

	return 0;
}

int motor_set_dir(uint16_t motor, bool way){

	if(motor == MOTOR_L){
		if(way == WAY_FORWARD)
			gpio_pin_set(dir_l, DIR_L_PIN_NUM, DIR_L_PIN_FORWARD);
		else
			gpio_pin_set(dir_l, DIR_L_PIN_NUM, !DIR_L_PIN_FORWARD);
	}
	else {
		if(way == WAY_FORWARD)
			gpio_pin_set(dir_r, DIR_R_PIN_NUM, DIR_R_PIN_FORWARD);
		else
			gpio_pin_set(dir_r, DIR_R_PIN_NUM, !DIR_R_PIN_FORWARD);
	}
	return 0;
}
