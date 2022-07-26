/*
 * rover.hpp
 *
 *  Created on: 26 Jul 2022
 *      Author: Shao Yuan
 */

#ifndef ROVER_HPP_
#define ROVER_HPP_

#include "l298n_motor.hpp"
#include "encoder.hpp"
#include "pid.hpp"

struct RoverResources{
  Motor motor_left;
  Motor motor_right;
  Encoder encoder_left;
  Encoder encoder_right;
  Pid pid_left;
  Pid pid_right;
};

struct RoverState{
  double target_rad_per_sec_left;
  double target_rad_per_sec_right;
  bool enable_pid;
  bool is_moving;
  uint32_t last_motor_command_millis;
  uint32_t next_pid_millis;
};


#endif /* ROVER_HPP_ */
