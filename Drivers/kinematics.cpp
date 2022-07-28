/*
 * kinematics.cpp
 *
 *  Created on: 28 Jul 2022
 *      Author: Shao Yuan
 */

#include "kinematics.hpp"
#include <algorithm>


#define _USE_MATH_DEFINES // for C++
#include <cmath>

Kinematics::Kinematics(int max_rpm, float wheel_diameter_m, float wheels_y_distance_m):
    max_rpm_(max_rpm),
    wheels_y_distance_m_(wheels_y_distance_m){
  wheel_circumference_m_ = wheel_diameter_m * M_PI;
}

Kinematics::Rpms Kinematics::get_rpms(Kinematics::Velocities vels){
  float tangential_m_sec = vels.angular_z_rad_sec * (wheels_y_distance_m_ / 2.0);

  float linear_x_m_min = vels.linear_x_m_sec * 60.0;
  float tangential_rad_min = tangential_m_sec * 60.0;

  float x_rpm = linear_x_m_min / wheel_circumference_m_;
  float tan_rpm = tangential_rad_min / wheel_circumference_m_;

  float a_x_rpm = fabs(x_rpm);
  float a_tan_rpm = fabs(tan_rpm);

  float xtan_sum = a_x_rpm + a_tan_rpm;

  //calculate the scale value how much each target velocity
  //must be scaled down in such cases where the total required RPM
  //is more than the motor's max RPM
  //this is to ensure that the required motion is achieved just with slower speed
  if(a_x_rpm >= max_rpm_ && vels.angular_z_rad_sec == 0)
  {
      float vel_scaler = max_rpm_ / a_x_rpm;
      x_rpm *= vel_scaler;
  }

  else if(xtan_sum >= max_rpm_)
  {
      float vel_scaler = max_rpm_ / xtan_sum;
      x_rpm *= vel_scaler;
      tan_rpm *= vel_scaler;
  }

  Kinematics::Rpms rpms;

  //calculate for the target motor RPM and direction
  //front-left motor
  rpms.motor_left = x_rpm - tan_rpm;
  rpms.motor_left = std::clamp(rpms.motor_left, -max_rpm_, max_rpm_);

  //front-right motor
  rpms.motor_right = x_rpm + tan_rpm;
  rpms.motor_right = std::clamp(rpms.motor_right, -max_rpm_, max_rpm_);

  return rpms;
}

Kinematics::Velocities Kinematics::get_velocities(Rpms rpms){
  Kinematics::Velocities vels;

  //convert average revolutions per minute to revolutions per second
  float x_rps = ((float)(rpms.motor_left + rpms.motor_right) / (float) TOTAL_WHEELS) / 60.0;
  vels.linear_x_m_sec = x_rps * wheel_circumference_m_;

  //convert average revolutions per minute to revolutions per second
  float tan_rps = ((float)(-rpms.motor_left + rpms.motor_right) / TOTAL_WHEELS) / 60.0;
  vels.angular_z_rad_sec =  (tan_rps * wheel_circumference_m_) / (wheels_y_distance_m_ / 2.0);

  return vels;
}



