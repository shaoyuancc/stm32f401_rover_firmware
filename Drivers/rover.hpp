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
#include "kinematics.hpp"

enum MotorControlMode : uint8_t{
  Stopped = 0,
  Pwm,
  Rpm,
  Velocity
};

struct RoverResources{
  Motor motor_left;
  Motor motor_right;
  Encoder encoder_left;
  Encoder encoder_right;
  Pid pid_left;
  Pid pid_right;
  Kinematics kinematics;
};

struct RoverState{
  Kinematics::Rpms target_rpms;
  MotorControlMode motor_control_mode;
  uint32_t last_motor_command_millis;
  uint32_t last_odom_millis;
  uint32_t next_pid_millis;
};


#endif /* ROVER_HPP_ */
