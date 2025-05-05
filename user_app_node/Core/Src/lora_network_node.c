/*
 * lora_network_node.c
 *
 *  Created on: Apr 11, 2025
 *      Author: PC
 */

#include "lora_network_node.h"
#include "bsp_sim.h"

extern lora_packet_t lora_receive_packet[NUMBER_OF_LORA_PACKET_BUFFER];
extern lora_packet_t lora_send_packet[NUMBER_OF_LORA_PACKET_BUFFER];
volatile uint8_t packet_received_count = 0;

volatile uint8_t connect_request_receive_flag = 0;
volatile uint8_t disconnect_request_receive_flag = 0;
volatile uint8_t read_request_receive_flag = 0;
extern volatile uint8_t test_warning_flag;

volatile uint8_t start_send_warning = 0;
uint8_t continue_send_warning = 0;
volatile uint8_t send_warning_done = 0;
volatile uint8_t send_warning_nak = 0;

uint8_t connect_packet_index = 0;
uint8_t disconnect_packet_index = 0;
uint8_t read_packet_index = 0;
uint8_t lr_nw_packet_id = 1;

extern UART_HandleTypeDef huart2;
extern lora_packet_t lora_receive_packet_buffer;
extern LoRa myLoRa;

void lora_network_init(){
	bsp_lora_init();
	/*if(bsp_sim_init_mqtt()){
		STM_LOG("Sim mqtt init success\n", 0, 22);
	}*/
}

void lora_network_send_warning(){
	uint32_t last_tick = 0;
	int packet_index;
	uint8_t send_warning_success = 1;
	while(send_warning_done == 0){
		if(start_send_warning == 1){
			while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			while(bsp_lora_check_cad() == 1);
			bsp_lora_set_receive_mode();
			uint8_t len_test = 10;
			uint8_t data_test[10] = {2, 1, 1, 0, 0, 1, 1, 1, 1, 1};
			STM_LOG("Start send warning\n", 0, 19);
			packet_index = bsp_lora_send_packet_to_device(GATEWAY_ID, LORA_CMD_WARNING, 0, data_test, len_test, 3, 0);

			continue_send_warning = 1;
			start_send_warning = 0;
			last_tick = HAL_GetTick();
		}

		else if(continue_send_warning == 1){
			if(HAL_GetTick() - last_tick >= 2000 || send_warning_nak == 1){
				STM_LOG("Send warning timeout\n", 0, 22);
				if(lora_send_packet[packet_index].ttl > 0){
					STM_LOG("Continue send warning\n", 0, 22);
					bsp_lora_resend_packet(&lora_send_packet[packet_index]);
				}
				else{
					STM_LOG("Cann't send warning\n", 0, 20);
					send_warning_success = 0;
					continue_send_warning = 0;
					send_warning_done = 1;
				}
				last_tick = HAL_GetTick();
				if(send_warning_nak == 1)
					send_warning_nak = 0;
			}
		}
	}

	if(send_warning_done == 1 && send_warning_success == 1){
		STM_LOG("Send warning sucessfully\n", 0, 25);
	}
	else{
		STM_LOG("Send warning failed\n", 0, 25);
	}
	send_warning_done = 0;
	send_warning_success = 1;

	if(packet_index != -1)
		lora_network_remove_packet_in_send_packet(packet_index);
}

void lora_network_send_response(uint8_t cmd){
	while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
	//while(bsp_lora_check_cad() == 1);
	bsp_lora_set_receive_mode();
	lr_nw_packet_id = (lr_nw_packet_id % (MAX_PACKET_ID)) + 1;
	int packet_index = -1;

	switch(cmd){
	case LORA_CMD_CONNECT:
		bsp_lora_send_packet(DEVICE_ID, GATEWAY_ID, lr_nw_packet_id, LORA_CMD_ACK, 0, NULL, 0, 3,
				lora_receive_packet[connect_packet_index].packet_id);
		packet_index = connect_packet_index;
		break;
	case LORA_CMD_DISCONNECT:
		bsp_lora_send_packet(DEVICE_ID, GATEWAY_ID, lr_nw_packet_id, LORA_CMD_ACK, 0, NULL, 0, 3,
				lora_receive_packet[disconnect_packet_index].packet_id);
		packet_index = disconnect_packet_index;
		break;
	case LORA_CMD_READ_DATA:
		uint8_t len_test = 10;
		uint8_t data_test[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

		bsp_lora_send_packet(DEVICE_ID, GATEWAY_ID, lr_nw_packet_id, LORA_CMD_ACK, 0, data_test, len_test, 3,
				lora_receive_packet[read_packet_index].packet_id);
		packet_index = read_packet_index;
		break;
	}

	if(packet_index != -1){
		for(uint8_t i = packet_index; i < NUMBER_OF_LORA_PACKET_BUFFER - 1; i++){
			if(lora_receive_packet[i].packet_id == 0)
				break;
			memcpy(&lora_receive_packet[i], &lora_receive_packet[i + 1], sizeof(lora_packet_t));
		}

		if(lora_receive_packet[NUMBER_OF_LORA_PACKET_BUFFER - 1].packet_id != 0)
			memset(&lora_receive_packet[NUMBER_OF_LORA_PACKET_BUFFER - 1], 0, sizeof(lora_packet_t));
	}

}

void lora_network_ack_cmd_handle(){
	uint8_t responsed = lora_receive_packet_buffer.responsed;
	int packet_index = lora_network_check_id_in_send_packet(responsed);

	if(packet_index != -1){
		uint8_t cmd = lora_receive_packet[packet_index].cmd;
		switch(cmd){
		case LORA_CMD_WARNING:
			send_warning_done = 1;
			break;
		}
	}
}

void lora_network_irq_handle(){
	STM_LOG("DIO0\n", 0, 5);
	while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
	uint8_t irqFlags = LoRa_read(&myLoRa, RegIrqFlags);
	if(irqFlags & 0x40) {
		STM_LOG("irq\n", 0, 4);
		bsp_lora_receive_packet();
		uint8_t packet_id = lora_receive_packet_buffer.packet_id;
		uint8_t cmd = lora_receive_packet_buffer.cmd;
		uint8_t device_id = lora_receive_packet_buffer.destination_id;
		STM_LOG("pid:%3d\n", packet_id, 8);
		// if there is no packet has the same pid or cmd field with recent received packet
		if((device_id == DEVICE_ID) &&
		(lora_network_check_id_in_receive_packet(packet_id) == -1 || lora_network_check_cmd_in_receive_packet(cmd) == -1)){
			int packet_index = lora_network_get_index_of_receive_packet();
			if(packet_index != -1){
				memcpy(&lora_receive_packet[packet_index], &lora_receive_packet_buffer, sizeof(lora_packet_t));

				switch(cmd){
				case LORA_CMD_CONNECT:
					if(connect_request_receive_flag == 0){
						STM_LOG("Set connect request flag\n", 0, 25);
						connect_request_receive_flag = 1;
						connect_packet_index = packet_index;
					}
					break;

				case LORA_CMD_DISCONNECT:
					if(disconnect_request_receive_flag == 0){
						STM_LOG("Set disconnect request flag\n", 0, 28);
						disconnect_request_receive_flag = 1;
						disconnect_packet_index = packet_index;
					}
					break;

				case LORA_CMD_READ_DATA:
					if(read_request_receive_flag == 0){
						STM_LOG("Set read request flag\n", 0, 22);
						read_request_receive_flag = 1;
						read_packet_index = packet_index;
					}
					break;

				case LORA_CMD_ACK:
					lora_network_ack_cmd_handle();
				}
				packet_received_count++;

			}else{
				HAL_UART_Transmit(&huart2, "lora_receive packet buffer full\n", 33, 2000);
			}
		}
	}
}

void lora_network_process(){
	if(connect_request_receive_flag == 1){
		lora_network_send_response(LORA_CMD_CONNECT);
		connect_request_receive_flag = 0;
	}

	if(disconnect_request_receive_flag == 1){
		lora_network_send_response(LORA_CMD_DISCONNECT);
		disconnect_request_receive_flag = 0;
	}

	if(read_request_receive_flag == 1){
		lora_network_send_response(LORA_CMD_READ_DATA);
		read_request_receive_flag = 0;
	}

	if(test_warning_flag == 1){
		test_warning_flag = 0;
		lora_network_send_warning();
	}
}
