/*
 * pid.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#include "pid.hpp"
#include <algorithm>

void Pid::reset(){
  prev_error_ = 0;
  integral_ = 0;
  derivative_ = 0;
}

double Pid::compute(double target, double measured){
  double error;
  double pid;

  error = target - measured;
  integral_ = std::clamp(integral_ + error, min_val_ * 2 , max_val_ * 2);
  derivative_ = error - prev_error_;

  if(target == 0 && error == 0)
  {
      integral_ = 0;
      derivative_ = 0;
  }

  pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
  double clamped_pid = std::clamp(pid, min_val_, max_val_);

  prev_error_ = error;

  return clamped_pid;
}


