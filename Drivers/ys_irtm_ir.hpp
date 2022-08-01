/*
 * ys_irtm_ir.hpp
 *
 *  Created on: Jul 31, 2022
 *      Author: Shao Yuan
 * Description: Driver for YS-IRTM Infrared Remote, encoder decoder transmitter receiver
 *   Baud Rate: 9600
 */

#ifndef YS_IRTM_IR_HPP_
#define YS_IRTM_IR_HPP_

#include "stm32f4xx_hal.h"

class YsIrtmIr {
private:
  UART_HandleTypeDef *p_uart_handle_;
  uint8_t tx_data_[5] = {0xA1,0xF1, 0x00, 0x00, 0x00};

public:
  uint8_t rx_data[3];

  YsIrtmIr() = delete;

  YsIrtmIr(UART_HandleTypeDef *p_uart_handle);

  void receive_data();

  void transmit_data(uint8_t *p_buff);

  void print_rx_data();
};

#endif /* YS_IRTM_IR_HPP_ */
