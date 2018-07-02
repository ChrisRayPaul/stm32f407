/*!
 * @file	dev_out_pwm.h
 * @brief
 *
 *
 *    Detailed description starts here.
 */


#ifndef _DEV_OUT_PWM_H_
#define _DEV_OUT_PWM_H_

#include "tim.h"

void dev_out_pwm_init(void);

extern uint32_t dev_out_pwm_set_period(uint32_t period);

extern uint32_t dev_out_pwm_set_pulse(uint32_t pulse);


#endif /* _DEV_OUT_PWM_H_ */
