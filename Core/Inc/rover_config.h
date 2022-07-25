/*
 * rover_config.h
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#ifndef INC_ROVER_CONFIG_H_
#define INC_ROVER_CONFIG_H_

#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant

#define MOTOR_MAX_RPM 40                   // motor's max RPM
#define MAX_RPM_RATIO 0.90                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
//#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate max RPM)
//#define MOTOR_POWER_MAX_VOLTAGE 12          // max voltage of the motor's power source (used to calculate max RPM)
//#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV 1441              // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.152                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.271            // distance between left and right wheels
#define PWM_MAX 255                          // Max pwm value
#define PWM_MIN -PWM_MAX
// INVERT MOTOR DIRECTIONS
#define MOTOR_LEFT_INV false
#define MOTOR_RIGHT_INV false

#endif /* INC_ROVER_CONFIG_H_ */
