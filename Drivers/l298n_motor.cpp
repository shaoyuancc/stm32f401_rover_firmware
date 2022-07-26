/*
 * motor.cpp
 *
 *  Created on: Jul 24, 2022
 *      Author: Shao Yuan
 */

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "l298n_motor.hpp"

// Spin motor forward with duty cycle = pwm/timer period.
// pwm    - The PWM value to set. This value should not be larger than the timer period.
void Motor::forward(uint16_t pwm){
  __HAL_TIM_SET_COMPARE(&timer_handle_, timer_channel_, pwm);

  HAL_GPIO_WritePin(p_in_a_gpio_port_, in_a_pin_, GPIO_PIN_SET);
  HAL_GPIO_WritePin(p_in_b_gpio_port_, in_b_pin_, GPIO_PIN_RESET);
}

// Spin motor in reverse with duty cycle = pwm/timer period.
// pwm    - The PWM value to set. This value should not be larger than the timer period.
void Motor::reverse(uint16_t pwm){
  __HAL_TIM_SET_COMPARE(&timer_handle_, timer_channel_, pwm);

  HAL_GPIO_WritePin(p_in_a_gpio_port_, in_a_pin_, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(p_in_b_gpio_port_, in_b_pin_, GPIO_PIN_SET);
}

// Starts the PWM necessary for the motor to run.
void Motor::activate(){
  brake();
  HAL_TIM_PWM_Start(&timer_handle_, timer_channel_);
  is_active_ = true;
}

// Stops the PWM necessary for the motor to run.
void Motor::deactivate(){
  brake();
  HAL_TIM_PWM_Stop(&timer_handle_, timer_channel_);
  is_active_ = false;
}

// Spin motor in direction dependent on sign (+/-) of pwm,
// with duty cycle = pwm/timer period.
// pwm    - The PWM value to set. This value should not be larger than the timer period.
void Motor::spin(int32_t pwm){
  if (invert_)
    pwm *= -1;

  if (pwm > 0) {
    forward((uint16_t) abs(pwm));
  } else if (pwm < 0) {
    reverse((uint16_t) abs(pwm));
  } else {
    brake();
  }
}

// Stop the motor from spinning.
void Motor::brake(){
  __HAL_TIM_SET_COMPARE(&timer_handle_, timer_channel_, 0);

  HAL_GPIO_WritePin(p_in_a_gpio_port_, in_a_pin_, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(p_in_b_gpio_port_, in_b_pin_, GPIO_PIN_RESET);
}
