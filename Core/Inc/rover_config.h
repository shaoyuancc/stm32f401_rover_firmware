/*
 * rover_config.h
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#ifndef INC_ROVER_CONFIG_H_
#define INC_ROVER_CONFIG_H_

// For RPM
//#define K_P 0.6                             // P constant
//#define K_I 0.8                             // I constant
//#define K_D 0.5                             // D constant

// For Rad Per Sec
#define K_P 70                         // P constant
#define K_I 45                             // I constant
#define K_D 30                           // D constant

#define MOTOR_MAX_RPM 40                   // motor's max RPM
#define MAX_RPM_RATIO 0.90                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define COUNTS_PER_REV 1441              // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.152                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.271            // distance between left and right wheels
#define ABS_PWM_MAX 255                          // Max abs pwm value
#define ABS_PWM_MIN 180

// Invert motor directions
#define MOTOR_LEFT_INV false
#define MOTOR_RIGHT_INV false

// Invert encoder directions
#define ENCODER_LEFT_INV true
#define ENCODER_RIGHT_INV false

#define PID_RATE_HZ           15     // Run the PID loop at 30 times per second

#define PID_INTERVAL_MILLIS 1000 / PID_RATE_HZ; //Convert the rate into an interval

#define AUTO_STOP_INTERVAL_MILLIS 5000  // Stop the robot if it hasn't received a movement command
                                        // in this number of milliseconds


#endif /* INC_ROVER_CONFIG_H_ */
