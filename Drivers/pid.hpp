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
    double abs_min_val_;
    double abs_max_val_;
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double derivative_;
    double prev_error_;

  public:
    Pid() = delete;

    Pid(double abs_min_val, double abs_max_val,
        double kp, double ki, double kd):
      abs_min_val_(abs_min_val),
      abs_max_val_(abs_max_val),
      kp_(kp),
      ki_(ki),
      kd_(kd) {
      reset();
    }

    void reset();

    double compute(double target, double measured);

    void update_constants(double kp, double ki, double kd);
};



#endif /* PID_HPP_ */
