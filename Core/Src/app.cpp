/*
 * app.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */
#include "main.h"
#include "string.h"
#include "stdio.h"

#include "rover_config.h"
#include "app.hpp"
#include "l298n_motor.hpp"
#include "gpio_output_device.hpp"
#include "encoder.hpp"
#include "pid.hpp"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

const char *DATA = "Hello from device\n";
uint8_t usb_buffer[64];

int32_t position = 0;
double current_rad_per_sec = 0.0;
double target_rad_per_sec = 0.0;
double pid_val = 0.0;

void start_app(){

  Motor motor_left = Motor(M1_L1_GPIO_Port, M1_L1_Pin,
                              M1_L2_GPIO_Port, M1_L2_Pin,
                              htim2, TIM_CHANNEL_3, false);

  Motor motor_right =  Motor(M2_L1_GPIO_Port, M2_L1_Pin,
                            M2_L2_GPIO_Port, M2_L2_Pin,
                            htim2, TIM_CHANNEL_1, false);

  GpioOutputDevice led = GpioOutputDevice(
                            LED_GPIO_Port, LED_Pin, ActiveLow);

  Encoder encoder_right = Encoder(htim3, COUNTS_PER_REV);

  Pid pid_right = Pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

  motor_right.brake();

  uint32_t i = 0;

  while (1)
    {
      if (i == 0) {
        led.on();
        target_rad_per_sec = 3.5;
      } else if (i == 100) {
        led.off();
        target_rad_per_sec = -3.5;
      }

      position = encoder_right.get_position();
      current_rad_per_sec = encoder_right.get_rad_per_sec();
      pid_val = pid_right.compute(target_rad_per_sec, current_rad_per_sec);
      motor_right.spin(pid_val);
      HAL_Delay(100);

      i ++;
      if (i > 200)
        i = 0;
    }
}




