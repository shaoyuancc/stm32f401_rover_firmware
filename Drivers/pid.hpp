/*
 * pid.hpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 *    Based on: LinoRobot2 by Juan Miguel Jimeno
 */

#ifndef PID_HPP_
#define PID_HPP_

class Pid {
  private:
    float abs_min_val_;
    float abs_max_val_;
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float derivative_;
    float prev_error_;

  public:
    Pid() = delete;

    Pid(float abs_min_val, float abs_max_val,
        float kp, float ki, float kd):
      abs_min_val_(abs_min_val),
      abs_max_val_(abs_max_val),
      kp_(kp),
      ki_(ki),
      kd_(kd) {
      reset();
    }

    void reset();

    float compute(float target, float measured);

    void update_constants(float kp, float ki, float kd);
};



#endif /* PID_HPP_ */
