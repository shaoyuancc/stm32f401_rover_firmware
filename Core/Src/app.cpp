/*
 * app.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */
#include <commands.h>
#include "string.h"
#include "stdio.h"

#include "main.h"
#include "rover_config.h"
#include "app.hpp"
#include "l298n_motor.hpp"
#include "gpio_output_device.hpp"
#include "encoder.hpp"
#include "pid.hpp"
#include "kinematics.hpp"
#include "rover.hpp"
#include "command_processor.hpp"
#include "usbd_cdc_if.h"
#include "ys_irtm_ir.hpp"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;

RoverResources *p_rover_resources = nullptr;

RoverState rover_state = RoverState {0};

void stop_and_reset_motors(){
  p_rover_resources->motor_left.brake();
  p_rover_resources->motor_right.brake();
  p_rover_resources->pid_left.reset();
  p_rover_resources->pid_right.reset();
  rover_state.target_rpms = Kinematics::Rpms{0};
  rover_state.motor_control_mode = MotorControlMode::Stopped;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart_handle){
  // Testing YS-IRTM Receiver
  if (p_rover_resources){
    p_rover_resources->ir.print_rx_data();
    p_rover_resources->ir.receive_data();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart_handle){
}

void do_pid(uint32_t millis){
  Kinematics::Rpms current_rpms = Kinematics::Rpms{
        .motor_left = p_rover_resources->encoder_left.get_rpm(),
        .motor_right = p_rover_resources->encoder_right.get_rpm()
  };

  switch(rover_state.motor_control_mode) {
    case MotorControlMode::Rpm: {
      printf("%c %f %f\n",
             ENCODERS_RPM,
             current_rpms.motor_left,
             current_rpms.motor_right);
      break;
    }
    case MotorControlMode::Velocity: {
      float dt_microseconds = ((float) (millis - rover_state.last_odom_millis)) / 1000.0;
      rover_state.last_odom_millis = millis;
      Kinematics::Velocities current_velocities = p_rover_resources->kinematics.get_velocities(current_rpms);
      printf("%c %f %f %f\n",
             ENCODERS_VELOCITIES,
             dt_microseconds,
             current_velocities.linear_x_m_sec,
             current_velocities.angular_z_rad_sec);
      break;
    }
    default: {
      break; // Do nothing
    }
  }

  p_rover_resources->motor_left.spin(
      p_rover_resources->pid_left.compute(
          rover_state.target_rpms.motor_left,
          current_rpms.motor_left)
  );

  p_rover_resources->motor_right.spin(
      p_rover_resources->pid_right.compute(
          rover_state.target_rpms.motor_right,
          current_rpms.motor_right)
  );

  // TODO: Handle overflow
  rover_state.next_pid_millis += PID_INTERVAL_MILLIS;
}

void start_app(){

  RoverResources resources = RoverResources {
    .motor_left = Motor(M1_L1_GPIO_Port, M1_L1_Pin,
                        M1_L2_GPIO_Port, M1_L2_Pin,
                        htim2, TIM_CHANNEL_3, MOTOR_LEFT_INV),

    .motor_right =  Motor(M2_L1_GPIO_Port, M2_L1_Pin,
                          M2_L2_GPIO_Port, M2_L2_Pin,
                          htim2, TIM_CHANNEL_1, MOTOR_RIGHT_INV),

    .encoder_left = Encoder(htim4, COUNTS_PER_REV, ENCODER_LEFT_INV),
    .encoder_right = Encoder(htim3, COUNTS_PER_REV, ENCODER_RIGHT_INV),

    .pid_left = Pid(ABS_PWM_MIN, ABS_PWM_MAX, K_P, K_I, K_D),
    .pid_right = Pid(ABS_PWM_MIN, ABS_PWM_MAX, K_P, K_I, K_D),
    .kinematics = Kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER_M, LR_WHEELS_DISTANCE_M),
    .ir = YsIrtmIr(&huart1),
  };

  p_rover_resources = &resources;

  GpioOutputDevice led = GpioOutputDevice(
                            LED_GPIO_Port, LED_Pin, ActiveLow);

  // Testing YS-IRTM Receiver
  p_rover_resources->ir.receive_data();

  // Testing YS-IRTM
  uint8_t data [3]={0x00, 0xEF, 0x03};

  while (1){
    // Testing YS-IRTM Transmitter
//    resources.ir.transmit_data(data);
//    HAL_Delay(500);


    if (rover_state.motor_control_mode == Stopped) {
      led.off();
      continue;
    }

    // Common for all moving states
    led.on();
    uint32_t millis = HAL_GetTick();       // TODO: Handle overflow
    uint32_t time_since_last_command = millis - rover_state.last_motor_command_millis;
    if (time_since_last_command > AUTO_STOP_INTERVAL_MILLIS) {
      stop_and_reset_motors();
      printf("motors timeout\n");
      continue;
    }

    if (rover_state.motor_control_mode == Pwm)
      continue;

    // For Motor Control Modes Rpm and Velocities
    if (millis > rover_state.next_pid_millis){
      do_pid(millis);
    }
  }
}





