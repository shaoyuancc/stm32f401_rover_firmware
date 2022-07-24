/*
 * motor.hpp
 *
 *  Created on: Jul 24, 2022
 *      Author: Shao Yuan
 */

#ifndef L298N_MOTOR_HPP_
#define L298N_MOTOR_HPP_

class Motor {
  private:
    GPIO_TypeDef* p_in_a_gpio_port_;
    uint16_t in_a_pin_;

    GPIO_TypeDef* p_in_b_gpio_port_;
    uint16_t in_b_pin_;

    TIM_HandleTypeDef timer_handle_;
    uint32_t timer_channel_;

    bool invert_;

    bool is_active_;

    void forward(uint16_t pwm);
    void reverse(uint16_t pwm);

  public:
    Motor() = delete; // Delete default class constructor

    Motor(GPIO_TypeDef* p_in_a_gpio_port, uint16_t in_a_pin,
          GPIO_TypeDef* p_in_b_gpio_port, uint16_t in_b_pin,
          TIM_HandleTypeDef timer_handle, uint32_t timer_channel,
          bool invert) :
            p_in_a_gpio_port_(p_in_a_gpio_port),
            in_a_pin_(in_a_pin),
            p_in_b_gpio_port_(p_in_b_gpio_port),
            in_b_pin_(in_b_pin),
            timer_handle_(timer_handle),
            timer_channel_(timer_channel),
            invert_(invert) {}

    ~Motor(){
      deactivate();
    }

    void activate();
    void deactivate();

    void spin(int32_t pwm);
    void brake();
};



#endif /* L298N_MOTOR_HPP_ */
