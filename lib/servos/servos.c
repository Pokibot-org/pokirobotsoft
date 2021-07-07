#include "servos.h"

LOG_MODULE_REGISTER(servos);

static servo_t servo_list[] = { // MUST BE IN THE SAME ORDER AS servo_names_t
    {1, DT_ALIAS_PWMS_SERVOS, NULL},
    {2, DT_ALIAS_PWMS_SERVOS, NULL},
    {3, DT_ALIAS_PWMS_SERVOS, NULL},
    {4, DT_ALIAS_PWMS_SERVOS, NULL}
};

#define NUMBER_OF_SERVOS ARRAY_SIZE(servo_list)

int servos_init()
{
	for (size_t i = 0; i < NUMBER_OF_SERVOS; i++)
	{
		servo_list[i].pwm_device = device_get_binding(servo_list[i].pwm_alias);
		if (!servo_list[i].pwm_device)
		{
			LOG_ERR("failed to get PWM SERVOS %u", i);
			return -1;
		}

		if (pwm_pin_set_usec(servo_list[i].pwm_device, servo_list[i].channel, PERIOD_SERVOS, 0, 0))
		{
			LOG_ERR("PWM %u set fails\n", i);
			return -2;
		}
	}

	LOG_INF("PWM SERVO initialized.");
	return 0;
}

int servos_set(servo_names_t name, uint16_t val_degree)
{
	if ((uint8_t)name > NUMBER_OF_SERVOS){
		LOG_ERR("Undefined servo in servo_list");
		return -1;
	}
	servo_t * servo = &servo_list[(uint8_t)name];
	if (servo->pwm_device == NULL)
	{
		LOG_ERR("Servo not initialized");
		return -1;
	}

	uint16_t out_val = MINPULSEWIDTH + ((MAXPULSEWIDTH - MINPULSEWIDTH) *  MIN(val_degree, 180))/180;
	LOG_DBG("PWM servo %s %d at speed %d %d", servo->pwm_alias, servo->channel, val_degree, out_val);
	if (pwm_pin_set_usec(servo->pwm_device, servo->channel, PERIOD_SERVOS, out_val, 0))
	{
		LOG_ERR("PWM servo %s %d set fails", servo->pwm_alias, servo->channel);
		return -1;
	}

	return 0;
}

void test_servo()
{
	servos_init();
	while (1)
	{
		for (size_t i = 0; i < 180; i+= 10)
		{
			servos_set(servo_front_l, i);
			k_sleep(K_MSEC(100));
		}
		for (size_t i = 0; i < 180; i+=10)
		{
			servos_set(servo_front_l, 180-i);
			k_sleep(K_MSEC(100));
		}
	}
}