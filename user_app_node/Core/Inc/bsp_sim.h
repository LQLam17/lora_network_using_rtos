/*
 * bsp_sim.h
 *
 *  Created on: Apr 15, 2025
 *      Author: PC
 */

#ifndef INC_BSP_SIM_H_
#define INC_BSP_SIM_H_

#include "stm32f4xx_hal.h"

#define MAX_SIM_RX_BUFFER_LENGTH 100
#define MAX_SIM_DATA_RX_BUFFER_LENGTH 200

#define SIM_MQTT_HOST "tcp://mqtt.eclipseprojects.io"
#define SIM_MQTT_PORT 1883

uint8_t bsp_sim_init();
uint8_t bsp_sim_init_mqtt();
uint8_t bsp_sim_sub(char *sub_topic);
uint8_t bsp_sim_unsub(char *unsub_topic);
uint8_t bsp_sim_pub(char *pub_topic, char *data);

#endif /* INC_BSP_SIM_H_ */
