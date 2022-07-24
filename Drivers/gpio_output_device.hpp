/*
 * gpio_output_device.hpp
 *
 *  Created on: Jul 25, 2022
 *      Author: chewchiashaoyuan
 */

#ifndef GPIO_OUTPUT_DEVICE_HPP_
#define GPIO_OUTPUT_DEVICE_HPP_

enum ActiveLogicLevel : uint8_t {
  ActiveLow = 0,
  ActiveHigh = 1
};

class GpioOutputDevice {
  private:
    GPIO_TypeDef* p_gpio_port_;
    uint16_t pin_;
    ActiveLogicLevel active_logic_level_;

  public:
    GpioOutputDevice() = delete;

    GpioOutputDevice(
        GPIO_TypeDef* p_gpio_port,
        int16_t pin,
        ActiveLogicLevel active_logic_level) :
          p_gpio_port_(p_gpio_port),
          pin_(pin),
          active_logic_level_(active_logic_level){}

    void on();
    void off();
    void toggle();
};



#endif /* GPIO_OUTPUT_DEVICE_HPP_ */
