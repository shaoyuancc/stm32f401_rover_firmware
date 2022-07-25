/*
 * app.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */
#include "main.h"
#include "string.h"
#include "stdio.h"
#include "app.hpp"
#include "l298n_motor.hpp"
#include "gpio_output_device.hpp"
#include "encoder.hpp"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

const uint32_t ENCODER_COUNTS_PER_REVOLUTION = 1441;
const char *DATA = "Hello from device\n";
uint8_t usb_buffer[64];

int32_t position = 0;
double rpm = 0.0;

void start_app(){

  Motor motor_left = Motor(M1_L1_GPIO_Port, M1_L1_Pin,
                              M1_L2_GPIO_Port, M1_L2_Pin,
                              htim2, TIM_CHANNEL_3, false);

  Motor motor_right =  Motor(M2_L1_GPIO_Port, M2_L1_Pin,
                            M2_L2_GPIO_Port, M2_L2_Pin,
                            htim2, TIM_CHANNEL_1, false);

  GpioOutputDevice led = GpioOutputDevice(
                            LED_GPIO_Port, LED_Pin, ActiveLow);

  Encoder encoder_right = Encoder(htim3, ENCODER_COUNTS_PER_REVOLUTION);

  motor_right.brake();
//  motor_right.spin(70);

  uint32_t i = 0;

  while (1)
    {
//      if (i == 0) {
//        led.on();
//        encoder_right.reset();
//        motor_right.spin(100);
//      } else if (i == 100) {
//        led.off();
//        encoder_right.reset();
//        motor_right.spin(-100);
//      }

      position = encoder_right.get_position();
      rpm = encoder_right.get_rpm();
      HAL_Delay(100);

//      i ++;
//      if (i > 200)
//        i = 0;
    }
}




