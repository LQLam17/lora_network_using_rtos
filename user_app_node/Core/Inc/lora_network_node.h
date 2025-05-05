/*
 * lora_network_node.h
 *
 *  Created on: Apr 11, 2025
 *      Author: PC
 */

#ifndef INC_LORA_NETWORK_NODE_H_
#define INC_LORA_NETWORK_NODE_H_

#include "bsp_lora.h"


void lora_network_irq_handle();
void lora_network_init();
void lora_network_process();


#endif /* INC_LORA_NETWORK_NODE_H_ */
