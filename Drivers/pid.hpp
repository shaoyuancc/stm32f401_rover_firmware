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
    double min_val_;
    double max_val_;
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double derivative_;
    double prev_error_;

    void reset();

  public:
    Pid() = delete;

    Pid(double min_val, double max_val, double kp, double ki, double kd):
      min_val_(min_val),
      max_val_(max_val),
      kp_(kp),
      ki_(ki),
      kd_(kd) {
      reset();
    }

    double compute(double target, double measured);

    void update_constants(double kp, double ki, double kd);
};



#endif /* PID_HPP_ */
