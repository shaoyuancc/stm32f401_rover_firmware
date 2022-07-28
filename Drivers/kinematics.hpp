/*
 * kinematics.hpp
 *
 *  Created on: 28 Jul 2022
 *      Author: Shao Yuan
 */

#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <stdint.h>

class Kinematics {
  private:
    float max_rpm_;
    float wheels_y_distance_m_;
    float wheel_circumference_m_;
    const uint8_t TOTAL_WHEELS = 2;

  public:
    struct Rpms {
      float motor_left;
      float motor_right;
    };

    struct Velocities {
      float linear_x_m_sec;
      float angular_z_rad_sec;
    };

    Kinematics() = delete;

    Kinematics(int max_rpm, float wheel_diameter_m, float wheels_y_distance_m);

    Rpms get_rpms(Velocities vels);

    Velocities get_velocities(Rpms rpms);
};



#endif /* KINEMATICS_HPP_ */
