/*
 * encoder.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: Shao Yuan
 */

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include "encoder.hpp"

// Starts the timer peripheral in encoder mode.
// Note that initialization must be done before this and is not handled by the driver.
void Encoder::activate(){
  if (is_active_)
    return;

  HAL_TIM_Encoder_Start(&timer_handle_, TIM_CHANNEL_ALL);
  reset();
  is_active_ = true;
}

void Encoder::deactivate(){
  if (!is_active_)
    return;

  HAL_TIM_Encoder_Stop(&timer_handle_, TIM_CHANNEL_ALL);
  is_active_ = false;
}

int32_t Encoder::get_position(){
  return (int32_t) (__HAL_TIM_GET_COUNTER(&timer_handle_) / 4);
}

// Gets rpm of the encoder based on the difference in count and time since the last time
// this method was called.
// Note: Ensure that the timer period is much greater than the inter-measurement period,
// to accurately account for wrap around of timer counter.
double Encoder::get_rpm(){
  uint32_t curr_count = __HAL_TIM_GET_COUNTER(&timer_handle_) / 4;
  uint32_t curr_time_millis = HAL_GetTick();
  uint32_t delta_time_millis = curr_time_millis - prev_time_millis_;

  if (delta_time_millis == 0)
    return 0.0;

  double delta_time_mins = (double) delta_time_millis / 60000;

  // Account for possibility of wrap around
  double delta_count_direct = (double) curr_count - prev_count_;
  double delta_count_wrap;

  bool did_increase = curr_count > prev_count_;
  if (did_increase) {
    delta_count_wrap = -1 * (double) (prev_count_ + (timer_period_ - curr_count)); // Actually decreasing but wrapped around.
  } else {
    delta_count_wrap = (double) (timer_period_ - prev_count_) + curr_count;// Actually increasing but wrapped around.
  }

  bool did_wrap = abs(delta_count_wrap) < abs(delta_count_direct);

  // The actual delta should be the smaller delta
  // As long as the timer period is much greater than the inter-measurement period.
  double delta_count = (did_wrap) ? delta_count_wrap : delta_count_direct;

  double rpm = (delta_count/counts_per_revolution_)/delta_time_mins;

  prev_time_millis_ = curr_time_millis;
  prev_count_ = curr_count;

  return rpm;
}

void Encoder::reset(){
  prev_count_ = 0;
  prev_time_millis_ =  HAL_GetTick();
  __HAL_TIM_SET_COUNTER(&timer_handle_, 0);
}
