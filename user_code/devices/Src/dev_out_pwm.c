/*!
 * @file	dev_out_pwm.c
 * @brief
 *
 *
 *    Detailed description starts here.
 */

#include "../Inc/dev_out_pwm.h"

#define PWM_MAX_PULSE	1000

#define PWM_MAX_PERIOD	60000
#define PWM_MIN_PERIOD	20

#define PWM_DEFAULT_PULSE	500
#define PWM_DEFAULT_PERIOD	1000

#define PWM_COUNT_PER_S		1000000
#define PWM_PRESCALER		84

void dev_out_pwm_init(void);

uint32_t dev_out_pwm_set_period(uint32_t period_hz) {
	uint32_t period = 0;

	if(period_hz > PWM_MAX_PERIOD || period_hz < PWM_MIN_PERIOD)
	{
		return 0;
	}
	else
	{
		period = (uint32_t)(PWM_COUNT_PER_S / period_hz);
	}

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = PWM_PRESCALER - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = period - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = period / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);
	/* after init we need start timer */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	return 1;
}

uint32_t dev_out_pwm_set_pulse(uint32_t pulse) {
	TIM_OC_InitTypeDef sConfigOC;

	if (pulse < 0 || pulse > PWM_MAX_PULSE) {
		return 0;
	} else {
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = pulse;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
		HAL_TIM_MspPostInit(&htim3);
		/* after init we need start timer */
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

		return 1;
	}
}

