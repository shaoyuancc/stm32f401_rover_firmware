/*
 * ys_irtm_ir.cpp
 *
 *  Created on: Jul 31, 2022
 *      Author: Shao Yuan
 */

#include "ys_irtm_ir.hpp"
#include "stm32f4xx_hal.h"
#include "stdio.h"

YsIrtmIr::YsIrtmIr(UART_HandleTypeDef *p_uart_handle) :
    p_uart_handle_(p_uart_handle) {
}

void YsIrtmIr::receive_data(){
  HAL_UART_Receive_IT(p_uart_handle_, rx_data, 3);
}

void YsIrtmIr::transmit_data(uint8_t *p_buff){
  tx_data_[2] = p_buff[0];
  tx_data_[3] = p_buff[1];
  tx_data_[4] = p_buff[2];
  HAL_UART_Transmit_IT(p_uart_handle_, tx_data_, sizeof(tx_data_));
}

void YsIrtmIr::print_rx_data(){
  printf("received: ");
  for (uint8_t i = 0; i < 3; i++){
    printf("%u ", rx_data[i]);
  }
  printf("\n");
}
