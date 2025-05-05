/*
 * bsp_sim.h
 *
 *  Created on: Apr 15, 2025
 *      Author: PC
 */

#include "bsp_sim.h"
#include "string.h"

const uint32_t sim_timeout = 5000;
char sim_apn[] = "m3-world";

extern UART_HandleTypeDef huart1;
uint8_t sim_rx_buffer[MAX_SIM_RX_BUFFER_LENGTH];
uint8_t sim_data_rx_buffer[MAX_SIM_DATA_RX_BUFFER_LENGTH];

void bsp_sim_transmit(char *cmd, uint32_t timeout){
	memset(sim_rx_buffer, 0, MAX_SIM_RX_BUFFER_LENGTH);
	HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), 1000);
	HAL_UART_Receive(&huart1, sim_rx_buffer, MAX_SIM_RX_BUFFER_LENGTH, timeout);
}

uint8_t bsp_sim_init(){
	uint8_t ATisOK = 0;
	uint8_t ATCREGisOK = 0;
	uint8_t ATCGREGisOK = 0;
	uint8_t ATPDPisOK = 0;

	uint8_t at_buffer[50];

	uint32_t previousTick =  HAL_GetTick();
	while(!ATisOK && HAL_GetTick() - previousTick < sim_timeout){
		bsp_sim_transmit("AT\r\n", 1000);
		if(strstr((char *)sim_rx_buffer, "OK")){
			ATisOK = 1;
		}
	}

	if(ATisOK){
		previousTick = HAL_GetTick();
		while(!ATCREGisOK && HAL_GetTick() - previousTick < sim_timeout){
			bsp_sim_transmit("AT+CREG\r\n", 2000);
			if(strstr((char *)sim_rx_buffer, "OK")){
				ATCREGisOK = 1;
			}
		}
	}

	if(ATCREGisOK){
		previousTick = HAL_GetTick();
		while(!ATCGREGisOK && HAL_GetTick() - previousTick < sim_timeout){
			bsp_sim_transmit("AT+CGREG\r\n", 2000);
			if(strstr((char *)sim_rx_buffer, "OK")){
				ATCGREGisOK = 1;
			}
		}
	}

	if(ATCGREGisOK){
		previousTick = HAL_GetTick();
		while(!ATPDPisOK && HAL_GetTick() - previousTick < sim_timeout){
			sprintf(at_buffer, "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", sim_apn);
			bsp_sim_transmit(at_buffer, 1000);
			if(strstr((char *)sim_rx_buffer, "OK")){
				ATCGREGisOK = 1;
			}
		}
	}

	if(ATCGREGisOK)
		return 1;

	return 0;
}

uint8_t bsp_sim_init_mqtt(){
	uint8_t mqtt_start_flag = 0;
	uint8_t mqtt_client_flag = 0;
	uint8_t mqtt_connect_flag = 0;

	uint8_t init_flag = bsp_sim_init();
	uint32_t previousTick = 0;

	uint8_t at_mqtt_buffer[50];
	if(init_flag){
		previousTick = HAL_GetTick();
		while(!mqtt_start_flag && HAL_GetTick() - previousTick < sim_timeout){
			bsp_sim_transmit("AT+CMQTTSTART\r\n", 1500);
			if(strstr((char *)sim_rx_buffer, "OK")){
				mqtt_start_flag = 1;
			}
		}
	}
	else{
		return;
	}

	if(mqtt_start_flag){
		previousTick = HAL_GetTick();
		while(!mqtt_client_flag && HAL_GetTick() - previousTick < sim_timeout){
			bsp_sim_transmit("AT+CMQTTACCQ=0,\"client0\"\r\n", 1500);
			if(strstr((char *)sim_rx_buffer, "OK")){
				mqtt_client_flag = 1;
			}
		}
	}

	if(mqtt_client_flag){
		previousTick = HAL_GetTick();
		while(!mqtt_connect_flag && HAL_GetTick() - previousTick < sim_timeout){
			sprintf(at_mqtt_buffer, "AT+CMQTTCONNECT=0,\"%s:%d\",60,1\r\n", SIM_MQTT_HOST, SIM_MQTT_PORT);
			bsp_sim_transmit(at_mqtt_buffer, 2000);
			if(strstr((char *)sim_rx_buffer, "OK")){
				mqtt_connect_flag = 1;
				return 1;
			}
		}
	}
	return 0;
}

uint8_t bsp_sim_sub(char *sub_topic){
	uint8_t mqtt_sub_flag = 0;
	uint8_t at_mqtt_buffer[50];

	uint32_t previousTick = HAL_GetTick();

	while(!mqtt_sub_flag && HAL_GetTick() - previousTick < sim_timeout){
		sprintf(at_mqtt_buffer, "AT+CMQTTSUB=0,%d,1\r\n", strlen(sub_topic));
		bsp_sim_transmit(at_mqtt_buffer, 500);
		sprintf(at_mqtt_buffer, "%s\r\n", sub_topic);
		bsp_sim_transmit(at_mqtt_buffer, 2000);
		if(strstr((char *)at_mqtt_buffer, "OK")){
			mqtt_sub_flag = 1;
			return 1;
		}
	}

	return 0;
}

uint8_t bsp_sim_unsub(char *unsub_topic){
	uint8_t mqtt_unsub_flag = 0;
	uint8_t at_mqtt_buffer[50];

	uint32_t previousTick = HAL_GetTick();

	while(!mqtt_unsub_flag && HAL_GetTick() - previousTick < sim_timeout){
		sprintf(at_mqtt_buffer, "AT+CMQTTUNSUB=0,%d,0\r\n", strlen(unsub_topic));
		bsp_sim_transmit(at_mqtt_buffer, 300);
		sprintf(at_mqtt_buffer, "%s\r\n", unsub_topic);
		bsp_sim_transmit(at_mqtt_buffer, 1000);

		if(strstr((char *)at_mqtt_buffer, "OK")){
			mqtt_unsub_flag = 1;
			return 1;
		}
	}

	return 0;
}

uint8_t bsp_sim_pub(char *pub_topic, char *data){
	uint8_t mqtt_topic_flag = 0;
	uint8_t mqtt_payload_flag = 0;
	uint8_t mqtt_pub_flag = 0;
	uint8_t at_mqtt_buffer[50];

	uint32_t previousTick = HAL_GetTick();

	while(!mqtt_topic_flag && HAL_GetTick() - previousTick < sim_timeout){
		sprintf(at_mqtt_buffer, "AT+CMQTTTOPIC=0,%d\r\n", strlen(pub_topic));
		bsp_sim_transmit(at_mqtt_buffer, 300);
		sprintf(at_mqtt_buffer, "%s\r\n", pub_topic);
		bsp_sim_transmit(at_mqtt_buffer, 1000);
		if(strstr((char *)at_mqtt_buffer, "OK")){
			mqtt_topic_flag = 1;
		}
	}

	if(mqtt_topic_flag){
		previousTick = HAL_GetTick();
		while(!mqtt_payload_flag && HAL_GetTick() - previousTick < sim_timeout){
			sprintf(at_mqtt_buffer, "AT+CMQTTPAYLOAD=0,%d\r\n", strlen(data));
			bsp_sim_transmit(at_mqtt_buffer, 300);
			sprintf(at_mqtt_buffer, "%s\r\n", data);
			bsp_sim_transmit(at_mqtt_buffer, 1000);
			if(strstr((char *)at_mqtt_buffer, "OK")){
				mqtt_payload_flag = 1;
			}
		}
	}

	if(mqtt_payload_flag){
		previousTick = HAL_GetTick();
		while(!mqtt_pub_flag && HAL_GetTick() - previousTick < sim_timeout){
			sprintf(at_mqtt_buffer, "AT+CMQTTPUB=0,1,%d\r\n", strlen(data));
			bsp_sim_transmit(at_mqtt_buffer, 2000);
			if(strstr((char *)at_mqtt_buffer, "OK")){
				mqtt_pub_flag = 1;
				return 1;
			}
		}
	}
	return 0;
}


