/*
 * bsp_lora.h
 *
 *  Created on: Feb 23, 2025
 *      Author: PC
 */

#ifndef INC_BSP_LORA_H_
#define INC_BSP_LORA_H_

#include "LoRa.h"
#include "stm32f4xx_hal.h"

#define GATEWAY_ID			0x00
#define DEVICE1_ID			0x01
#define DEVICE2_ID			0x02

#define MAX_PACKET_ID		250
#define MAX_PACKET_LENGTH 	200
#define MAX_PAYLOAD_LENGTH	32

#define MAX_NODE_PACKET_ITEMS	3

typedef enum lora_command{
	LORA_CMD_CONNECT = 0,
	LORA_CMD_DISCONNECT,
	LORA_CMD_WRITE_DATA,
	LORA_CMD_READ_DATA,
	LORA_CMD_PROGRAM,
	LORA_CMD_ACK,
	LORA_CMD_NACK,
	LORA_CMD_WARNING
}lora_command_e;

typedef struct __attribute__((packed)) lora_packet{
	uint8_t packet_id;						// packet's id
	uint8_t source_id;						// source device's id
	uint8_t destination_id;					// destination device's id
	uint8_t ttl;							// time to live (sub 1 each time send this message)
	uint8_t responsed;						// indicate if this packet is responsed, if this message is responsed the responsed's value of received packet will be equal to the transmited packet id
	//uint32_t time_stamp;					// the time when sending this message
	uint8_t cmd;							// command
	uint32_t mem_addr;
	uint32_t checksum;						// 4 bytes check sum
	uint8_t payload_length;					// payload's length
	uint8_t payload[MAX_PAYLOAD_LENGTH];	// payload
}lora_packet_t;

typedef struct lora_node{
	uint8_t id;
	uint8_t class;
	uint8_t connected;
	//uint8_t requesting_connect;
	//uint8_t requesting_disconnect;
	volatile uint8_t sending_data;

	uint8_t error;
	lora_packet_t last_lora_send_packet[MAX_NODE_PACKET_ITEMS];
	lora_packet_t last_lora_receive_packet[MAX_NODE_PACKET_ITEMS];
}lora_node_t;

extern LoRa myLoRa;

extern lora_packet_t lora_send_packet_buffer;
extern lora_packet_t lora_receive_packet_buffer;


int bsp_lora_spi_is_free();

int bsp_lora_check_cmd_in_node_send_packets(lora_node_t *lora_node, uint8_t lora_cmd);
void bsp_lora_remove_packet_from_node_send_packets(lora_node_t *lora_node, uint8_t index);

int bsp_lora_get_node_receive_packet_index(lora_node_t *lora_node);
int bsp_lora_check_cmd_in_node_receive_packets(lora_node_t *lora_node, uint8_t lora_cmd);
void bsp_lora_remove_packet_from_node_receive_packets(lora_node_t *lora_node, uint8_t index);



void bsp_lora_init();

void bsp_lora_set_receive_mode();

void bsp_lora_set_cad_mode();

void LoRa_CAD_Detect();

void bsp_lora_send_packet_to_node(lora_node_t *des_node, uint8_t cmd, uint32_t mem_addr, uint8_t *data, uint8_t len, uint8_t ttl);

void bsp_lora_send_request_connect(lora_node_t *des_node);

void bsp_lora_send_receive_packet();

void bsp_lora_resend_packet(lora_packet_t *lora_packet);

void bsp_lora_receive_packet();

void bsp_lora_irq_handle();



#endif /* INC_BSP_LORA_H_ */
