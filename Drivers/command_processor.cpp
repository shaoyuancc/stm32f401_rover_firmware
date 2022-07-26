/*
 * commands.cpp
 *
 *  Created on: 26 Jul 2022
 *      Author: Shao Yuan
 */

#include <commands.h>
#include "command_processor.hpp"
#include "app.hpp"
#include "stdio.h"
#include "rover.hpp"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"

extern RoverResources *p_rover_resources;
extern RoverState rover_state;

void run_command(char cmd, char *p_arg1, char *p_arg2){

  switch(cmd) {
    case ENCODERS_RAD_PER_SEC:
      printf("%c %f %f\n",
             ENCODERS_RAD_PER_SEC,
             p_rover_resources->encoder_left.get_rad_per_sec(),
             p_rover_resources->encoder_right.get_rad_per_sec());
      break;
    case MOTORS_RAD_PER_SEC:
      rover_state.target_rad_per_sec_left = atof(p_arg1);
      rover_state.target_rad_per_sec_right = atof(p_arg2);
      rover_state.last_motor_command_millis = HAL_GetTick();
      if (rover_state.target_rad_per_sec_left == 0 &&
          rover_state.target_rad_per_sec_right == 0) {
        stop_and_reset_motors();
      }else{
        p_rover_resources->pid_left.reset();
        p_rover_resources->pid_right.reset();
        rover_state.next_pid_millis = HAL_GetTick();
      }
      rover_state.enable_pid = true;
      rover_state.is_moving = true;
      printf("%c OK\n", MOTORS_RAD_PER_SEC);
      break;
    case MOTORS_RAW_PWM:
      int16_t arg1 = atoi(p_arg1);
      int16_t arg2 = atoi(p_arg2);
      rover_state.last_motor_command_millis = HAL_GetTick();
      rover_state.enable_pid = false;
      if (arg1 == 0 && arg2 == 0) {
        stop_and_reset_motors();
      }
      p_rover_resources->motor_left.spin(arg1);
      p_rover_resources->motor_right.spin(arg2);
      rover_state.is_moving = true;
      printf("%c OK %d %d\n", MOTORS_RAW_PWM, arg1, arg2);
      break;
  };
}

void process_command(uint8_t* p_buff, uint32_t *p_len){

  // A pair of variables to help parse serial commands
  int arg = 0;
  int arg_index = 0;

  // Variable to hold an input character
  char chr;

  // Variable to hold the current single-character command
  char cmd;

  // Character arrays to hold the first and second arguments
  char argv1[16];
  char argv2[16];

  for (uint8_t i = 0; i < *p_len; i++){
    chr = p_buff[i];

    if (chr == '\n'){
      if (arg == 1) argv1[arg_index] = '\0';
      else if (arg == 2) argv2[arg_index] = '\0';
      run_command(cmd, argv1, argv2);
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[arg_index] = '\0';
        arg = 2;
        arg_index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[arg_index] = chr;
        arg_index++;
      }
      else if (arg == 2) {
        argv2[arg_index] = chr;
        arg_index++;
      }
    }
  }
}

int _write(int file, char *ptr, int len)
{
  CDC_Transmit_FS((uint8_t *) ptr, len);
  return len;
}


