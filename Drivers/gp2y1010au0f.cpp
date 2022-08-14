/*
 * gp2y1010au0f.cpp
 *
 *  Created on: 2 Aug 2022
 *      Author: Shao Yuan
 */
#undef USE_ADC
#ifdef USE_ADC
#include"gp2y1010au0f.hpp"

uint32_t Gp2y1010au0f::read_dust_raw_blocking(){
  uint32_t adc_val;

  led_.on();
  HAL_Delay(1);
  // ADC Poll
  HAL_ADC_Start(p_adc_handle_);
  HAL_ADC_PollForConversion(p_adc_handle_, 100);
  adc_val = HAL_ADC_GetValue(p_adc_handle_);
  HAL_ADC_Stop(p_adc_handle_);
  led_.off();

  return adc_val;
}
#endif
