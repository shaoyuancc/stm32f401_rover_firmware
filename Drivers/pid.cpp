/*
 * pid.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */
#undef PID_TUNING

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
  double clamped_pid;

  error = target - measured;
  integral_ = std::clamp(integral_ + error, abs_max_val_ * -2 , abs_max_val_ * 2);
  derivative_ = error - prev_error_;

  pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);

  clamped_pid = (pid > 0) ?
      std::clamp(pid, abs_min_val_, abs_max_val_) :
      std::clamp(pid, -abs_max_val_, -abs_min_val_);

  prev_error_ = error;

#ifdef PID_TUNING
  printf("e:%.2f r:%.2f\n",
               error * 10, measured * 10);
#endif

  return clamped_pid;
}


