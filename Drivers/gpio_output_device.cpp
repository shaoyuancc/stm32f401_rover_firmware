/*
 * gpio_output_device.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#include "stm32f4xx_hal.h"
#include "gpio_output_device.hpp"

void GpioOutputDevice::on(){
  switch (active_logic_level_) {
    case ActiveLow:
      HAL_GPIO_WritePin(p_gpio_port_, pin_, GPIO_PIN_RESET);
      break;
    case ActiveHigh:
    default:
      HAL_GPIO_WritePin(p_gpio_port_, pin_, GPIO_PIN_SET);
      break;
  }
}

void GpioOutputDevice::off(){
  switch (active_logic_level_) {
    case ActiveLow:
      HAL_GPIO_WritePin(p_gpio_port_, pin_, GPIO_PIN_SET);
      break;
    case ActiveHigh:
    default:
      HAL_GPIO_WritePin(p_gpio_port_, pin_, GPIO_PIN_RESET);
      break;
  }
}

void GpioOutputDevice::toggle(){
  HAL_GPIO_TogglePin(p_gpio_port_, pin_);
}

