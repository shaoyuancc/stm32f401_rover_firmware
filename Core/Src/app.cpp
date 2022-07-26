/*
 * app.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */
#include <commands.h>
#include "string.h"
#include "stdio.h"

#include "main.h"
#include "rover_config.h"
#include "app.hpp"
#include "l298n_motor.hpp"
#include "gpio_output_device.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "rover.hpp"
#include "command_processor.hpp"
#include "usbd_cdc_if.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

RoverResources *p_rover_resources = nullptr;

RoverState rover_state = RoverState {
  .target_rad_per_sec_left = 0,
  .target_rad_per_sec_right = 0,
  .enable_pid = false,
  .is_moving = false,
  .last_motor_command_millis = AUTO_STOP_INTERVAL_MILLIS,
  .next_pid_millis = 0,
};

void stop_and_reset_motors(){
  p_rover_resources->motor_left.brake();
  p_rover_resources->motor_right.brake();
  p_rover_resources->pid_left.reset();
  p_rover_resources->pid_right.reset();
  rover_state.target_rad_per_sec_left = 0;
  rover_state.target_rad_per_sec_right = 0;
  rover_state.enable_pid = false;
  rover_state.is_moving = false;
}

void do_pid(){
  double current_rad_per_sec_left = p_rover_resources->encoder_left.get_rad_per_sec();
  double current_rad_per_sec_right = p_rover_resources->encoder_right.get_rad_per_sec();

  printf("%c %f %f\n",
       ENCODERS_RAD_PER_SEC,
       current_rad_per_sec_left,
       current_rad_per_sec_right);

  p_rover_resources->motor_left.spin(
      p_rover_resources->pid_left.compute(
          rover_state.target_rad_per_sec_left,
          current_rad_per_sec_left)
  );

  p_rover_resources->motor_right.spin(
      p_rover_resources->pid_right.compute(
          rover_state.target_rad_per_sec_right,
          current_rad_per_sec_right)
  );

  // TODO: Handle overflow
  rover_state.next_pid_millis += PID_INTERVAL_MILLIS;
}

void start_app(){

  RoverResources resources = RoverResources {
    .motor_left = Motor(M1_L1_GPIO_Port, M1_L1_Pin,
                        M1_L2_GPIO_Port, M1_L2_Pin,
                        htim2, TIM_CHANNEL_3, MOTOR_LEFT_INV),

    .motor_right =  Motor(M2_L1_GPIO_Port, M2_L1_Pin,
                          M2_L2_GPIO_Port, M2_L2_Pin,
                          htim2, TIM_CHANNEL_1, MOTOR_RIGHT_INV),

    .encoder_left = Encoder(htim4, COUNTS_PER_REV, ENCODER_LEFT_INV),
    .encoder_right = Encoder(htim3, COUNTS_PER_REV, ENCODER_RIGHT_INV),

    .pid_left = Pid(ABS_PWM_MIN, ABS_PWM_MAX, K_P, K_I, K_D),
    .pid_right = Pid(ABS_PWM_MIN, ABS_PWM_MAX, K_P, K_I, K_D)
  };

  p_rover_resources = &resources;

  GpioOutputDevice led = GpioOutputDevice(
                            LED_GPIO_Port, LED_Pin, ActiveLow);

  while (1)
    {
      // TODO: Handle overflow
      uint32_t millis = HAL_GetTick();

      if (rover_state.enable_pid && millis > rover_state.next_pid_millis){
        do_pid();
      }

      uint32_t time_since_last_command = millis - rover_state.last_motor_command_millis;

      if (rover_state.is_moving && time_since_last_command > AUTO_STOP_INTERVAL_MILLIS) {
        stop_and_reset_motors();
        printf("motors timeout\n");
      }

      if (rover_state.is_moving){
        led.on();
      } else {
        led.off();
      }
    }
}





