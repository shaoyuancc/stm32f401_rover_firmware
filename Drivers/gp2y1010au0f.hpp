/*
 * gp2y1010au0f.hpp
 *
 *  Created on: 2 Aug 2022
 *      Author: Shao Yuan
 */

#ifndef GP2Y1010AU0F_HPP_
#define GP2Y1010AU0F_HPP_

#include "gpio_output_device.hpp"
#include "stm32f4xx_hal.h"

class Gp2y1010au0f {
  private:
    GpioOutputDevice led_;
    ADC_HandleTypeDef* p_adc_handle_;
  public:
    Gp2y1010au0f() = delete;

    Gp2y1010au0f(GPIO_TypeDef* p_led_gpio_port, int16_t led_pin, ADC_HandleTypeDef* p_adc_handle) :
      led_(GpioOutputDevice(p_led_gpio_port, led_pin, ActiveLow)),
      p_adc_handle_(p_adc_handle) {}

    uint32_t read_dust_raw_blocking();

//    uint32_t read_dust_raw();
};



#endif /* GP2Y1010AU0F_HPP_ */
