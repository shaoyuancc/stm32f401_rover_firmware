/*
 * command_processor.hpp
 *
 *  Created on: 26 Jul 2022
 *      Author: Shao Yuan
 */

#ifndef COMMAND_PROCESSOR_HPP_
#define COMMAND_PROCESSOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


void process_command(uint8_t* p_buff, uint32_t *p_len);

int _write(int file, char *ptr, int len);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_PROCESSOR_HPP_ */
