/*
 * bsp_lora.c
 *
 *  Created on: Feb 23, 2025
 *      Author: PC
 */

#include "bsp_lora.h"
#include "string.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

LoRa myLoRa = {0};

lora_packet_t lora_send_packet_buffer = {0};
lora_packet_t lora_receive_packet_buffer = {0};


void bsp_lora_enter_safe_mode(){
	NVIC_DisableIRQ(OTG_FS_IRQn);
	NVIC_DisableIRQ(EXTI0_IRQn);
	NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_DisableIRQ(EXTI2_IRQn);
	NVIC_DisableIRQ(EXTI3_IRQn);
}

void bsp_lora_exit_safe_mode(){
	NVIC_EnableIRQ(OTG_FS_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
}


void STM_LOG(char *str, int c, uint8_t size){
	char *buf = (char *)malloc(size * sizeof(char));
	sprintf(buf, str, c);
	HAL_UART_Transmit(&huart2, buf, size, 1000);
	free(buf);
}

int bsp_lora_spi_is_free(){
	if( (hspi1.Instance->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) == SPI_SR_TXE )
		return 1;
	return 0;
}


void bsp_lora_set_receive_mode(){
	bsp_lora_enter_safe_mode();
	LoRa_startReceiving(&myLoRa);
	bsp_lora_exit_safe_mode();
}

void bsp_lora_set_cad_mode(){
	bsp_lora_enter_safe_mode();
	LoRa_startCAD(&myLoRa);
	bsp_lora_exit_safe_mode();
}

/**
 * check if the bus is free or busy
 * @return: 1-bus is busy
 * 			0-bus is free
 * **/
uint8_t bsp_lora_check_cad() {
	bsp_lora_set_cad_mode();
	uint32_t startTime = HAL_GetTick();
    while (!(LoRa_read(&myLoRa, RegIrqFlags) & 0x04)){
    	if (HAL_GetTick() - startTime > 1000) { // Timeout sau 500ms
    		return 0; // Không phát hiện tín hiệu LoRa
    	}
    } // Chờ CAD hoàn thành
    uint8_t cadDetected = LoRa_read(&myLoRa, RegIrqFlags) & 0x01;
    LoRa_write(&myLoRa, RegIrqFlags, 0xFF); // Xóa cờ ngắt
    return cadDetected;
}

void bsp_lora_init(){
	// Initiate lora gateway object
	myLoRa = newLoRa();

	myLoRa.CS_port         = SPI1_NSS_GPIO_Port;
	myLoRa.CS_pin          = SPI1_NSS_Pin;
	myLoRa.reset_port      = LORA_RST_GPIO_Port;
	myLoRa.reset_pin       = LORA_RST_Pin;
	myLoRa.DIO0_port       = LORA_DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = LORA_DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;

	myLoRa.frequency             = 433;
	myLoRa.spredingFactor        = SF_7;
	myLoRa.bandWidth             = BW_250KHz;
	myLoRa.crcRate               = CR_4_5;
	myLoRa.power                 = POWER_17db;
	myLoRa.overCurrentProtection = 150;
	myLoRa.preamble              = 10;



	LoRa_reset(&myLoRa);
	if(LoRa_init(&myLoRa)==LORA_OK){
		HAL_UART_Transmit(&huart2, (uint8_t *)"LoRa Init Success\n", 18, 2000);
	}

	else{
		HAL_UART_Transmit(&huart2, (uint8_t *)"LoRa Init Failed\n", 18, 2000);
	}

	LoRa_startReceiving(&myLoRa);

	// Initiate lora node objects
	/*lora_node1.id = DEVICE1_ID;
	lora_node2.id = DEVICE2_ID;
	memset(&lora_node1.last_lora_send_packet, 0, sizeof(lora_packet_t));
	memset(&lora_node2.last_lora_send_packet, 0, sizeof(lora_packet_t));
	memset(&lora_node1.last_lora_receive_packet, 0, sizeof(lora_packet_t));
	memset(&lora_node2.last_lora_receive_packet, 0, sizeof(lora_packet_t));*/
}

uint32_t checksum_calculate(uint8_t *data, uint8_t len){
	if(data != NULL){
		uint32_t checksum = 0;
		for(int i = 0; i < len; i++){
			checksum += *(data + i);
		}
		return checksum;
	}
	return 0;
}

/**
 * send packets to destination device
 * @param des_id: destination id
 * @param cmd: command send to destination
 * @param data: pointer to the sending data
 * @param len: data's length
 */
void bsp_lora_send_packet(uint8_t source_id, uint8_t des_id, uint8_t packet_id, uint8_t cmd, uint32_t mem_addr, uint8_t *data, uint8_t len, uint8_t ttl){
	bsp_lora_enter_safe_mode();

	lora_send_packet_buffer.packet_id = packet_id;
	lora_send_packet_buffer.source_id = source_id;
	lora_send_packet_buffer.destination_id = des_id;
	lora_send_packet_buffer.ttl = ttl;
	lora_send_packet_buffer.responsed = 0;
	lora_send_packet_buffer.cmd = cmd;
	lora_send_packet_buffer.payload_length = len;
	lora_send_packet_buffer.mem_addr = mem_addr;
	lora_send_packet_buffer.checksum = checksum_calculate(data, len);

	memcpy(lora_send_packet_buffer.payload, data, len);

	uint8_t *buffer = (uint8_t *) (&lora_send_packet_buffer);

	LoRa_transmit(&myLoRa, buffer, sizeof(lora_packet_t), 4000);

	STM_LOG("pid: %3d\n", packet_id, 9);

	bsp_lora_exit_safe_mode();
}

int bsp_lora_get_node_send_packet_index(lora_node_t *lora_node){
	for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++){
		if(lora_node->last_lora_send_packet[i].packet_id == 0)
			return i;
	}
	return -1;
}

int bsp_lora_check_cmd_in_node_send_packets(lora_node_t *lora_node, uint8_t lora_cmd){
	for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++){
		if(lora_node->last_lora_send_packet[i].packet_id != 0 && lora_node->last_lora_send_packet[i].cmd == lora_cmd)
			return i;
	}
	return -1;
}

void bsp_lora_remove_packet_from_node_send_packets(lora_node_t *lora_node, uint8_t index){
	for(uint8_t i = index; i < MAX_NODE_PACKET_ITEMS - 1; i++){
		memcpy(&lora_node->last_lora_send_packet[i], &lora_node->last_lora_send_packet[i + 1], sizeof(lora_packet_t));
	}
	memset(&lora_node->last_lora_send_packet[MAX_NODE_PACKET_ITEMS - 1], 0, sizeof(lora_packet_t));
}

int bsp_lora_get_node_receive_packet_index(lora_node_t *lora_node){
	for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++){
		if(lora_node->last_lora_receive_packet[i].packet_id == 0)
			return i;
	}
	return -1;
}

int bsp_lora_get_id_in_node_receive_packets(lora_node_t *lora_node, uint8_t packet_id){
	for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++){
		if(lora_node->last_lora_receive_packet[i].packet_id == packet_id)
			return i;
	}
	return -1;
}

int bsp_lora_check_cmd_in_node_receive_packets(lora_node_t *lora_node, uint8_t lora_cmd){
	for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++){
		if(lora_node->last_lora_receive_packet[i].packet_id != 0 && lora_node->last_lora_receive_packet[i].cmd == lora_cmd)
			return i;
	}
	return -1;
}

void bsp_lora_remove_packet_from_node_receive_packets(lora_node_t *lora_node, uint8_t index){
	for(uint8_t i = index; i < MAX_NODE_PACKET_ITEMS - 1; i++){
		memcpy(&lora_node->last_lora_receive_packet[i], &lora_node->last_lora_receive_packet[i + 1], sizeof(lora_packet_t));
	}
	memset(&lora_node->last_lora_receive_packet[MAX_NODE_PACKET_ITEMS - 1], 0, sizeof(lora_packet_t));
}

/**
 * Send packet to a specific node
 * **/
void bsp_lora_send_packet_to_node(lora_node_t *des_node, uint8_t cmd, uint32_t mem_addr, uint8_t *data, uint8_t len, uint8_t ttl){
	if(bsp_lora_check_cmd_in_node_send_packets(des_node, cmd) == -1){
		if(cmd == LORA_CMD_CONNECT){
			if(des_node->connected == 0){
				HAL_UART_Transmit(&huart2, "connecting\n", 12, 2000);
				bsp_lora_send_packet(GATEWAY_ID, des_node->id, ((lora_send_packet_buffer.packet_id + 1) % (MAX_PACKET_ID + 1)) + 1
								, LORA_CMD_CONNECT, 0, NULL, 0, 3);
				HAL_UART_Transmit(&huart2, "sendcn\n", 7, 2000);
			}
			else{
				HAL_UART_Transmit(&huart2, "Node is connected\n", 19, 2000);
			}
		}

		else if(cmd == LORA_CMD_DISCONNECT){
			if(des_node->connected == 1){
				bsp_lora_send_packet(GATEWAY_ID, des_node->id, ((lora_send_packet_buffer.packet_id + 1) % (MAX_PACKET_ID + 1)) + 1
											, LORA_CMD_DISCONNECT, 0, NULL, 0, 3);
			}
			else{
				HAL_UART_Transmit(&huart2, "Node is disconnected\n", 21, 2000);
			}
		}

		else{
			HAL_UART_Transmit(&huart2, "read request\n", 13, 2000);
			bsp_lora_send_packet(GATEWAY_ID, des_node->id, ((lora_send_packet_buffer.packet_id + 1) % (MAX_PACKET_ID + 1)) + 1,
					cmd, mem_addr, data, len, ttl);
		}

		if(bsp_lora_get_node_send_packet_index(des_node) != -1){
			uint8_t i = bsp_lora_get_node_send_packet_index(des_node);
			memcpy(&des_node->last_lora_send_packet[i], &lora_send_packet_buffer, sizeof(lora_packet_t));
			des_node->last_lora_send_packet[i].ttl--;
		}
	}

}

/**
 * Resend a specific lora packet
 * **/
void bsp_lora_resend_packet(lora_packet_t *lora_packet){
	if(lora_packet->ttl > 0){
		bsp_lora_send_packet(GATEWAY_ID, lora_packet->destination_id, lora_packet->packet_id, lora_packet->cmd,
				lora_packet->mem_addr, lora_packet->payload, lora_packet->payload_length, lora_packet->ttl);
		lora_packet->ttl--;
	}
}

/*void bsp_lora_send_request_connect(lora_node_t *des_node){
	if( (des_node->connected == 0) && (des_node->sending_data == 0) ){
		bsp_lora_send_packet(GATEWAY_ID, des_node->id, (lora_send_packet_buffer.packet_id + 1) % (MAX_PACKET_ID + 1)
				, LORA_CMD_CONNECT, 0, NULL, 0, 3);
		memcpy(&des_node->last_lora_send_packet, &lora_send_packet_buffer, sizeof(lora_packet_t));
		des_node->last_lora_send_packet.ttl--;
	}
}

void bsp_lora_send_request_disconnect(lora_node_t *des_node){
	if( (des_node->connected == 1) && (des_node->sending_data == 0) ){
		bsp_lora_send_packet(GATEWAY_ID, des_node->id, (lora_send_packet_buffer.packet_id + 1) % (MAX_PACKET_ID + 1)
						, LORA_CMD_DISCONNECT, 0, NULL, 0, 3);
		memcpy(&des_node->last_lora_receive_packet, &lora_send_packet_buffer, sizeof(lora_packet_t));
		des_node->last_lora_send_packet.ttl--;
	}
}*/

/*void bsp_lora_write_reg(lora_node_t *des_node, uint32_t mem_addr, uint8_t *data, uint8_t len){
	if( ((des_node->connected == 1) && (des_node->sending_data == 0)) ){
		des_node->sending_data = 1;
		des_node->last_lora_send_packet = bsp_lora_send_packet(GATEWAY_ID, des_node->id, LORA_CMD_WRITE_REG, mem_addr, data, len, 3);
		//memcpy(des_node->last_lora_send_packet, lora_send_packet_buffer, sizeof(lora_packet_t));
		des_node->last_lora_send_packet.ttl--;
	}
}*/

/*void bsp_lora_resend_packet(lora_packet_t *lora_packet){

}*/

/**
 * A function used to handle payload received from other devices
 */
__weak void bsp_lora_handle_payload(uint8_t *data){

}


/**
 * A function used to receive packet from other devices
 */
void bsp_lora_receive_packet(){
	uint8_t u8lora_receive_packet_buffer[MAX_PACKET_LENGTH] = {0};
	LoRa_receive(&myLoRa, u8lora_receive_packet_buffer, MAX_PACKET_LENGTH);
	memcpy(&lora_receive_packet_buffer, u8lora_receive_packet_buffer, sizeof(lora_packet_t));
}

