/*
 * lora_network.c
 *
 *  Created on: Mar 21, 2025
 *      Author: PC
 */

#include "lora_network.h"
#include "usbd_cdc_if.h"


uint8_t lr_nw_node_quantity = 0;
uint8_t lr_nw_connected_node_quantity = 0;
uint8_t lr_nw_last_connected_node_quantity = 0;
uint8_t lr_nw_node_index = 0;
uint8_t lr_nw_node_index1 = 0;
uint8_t lr_nw_send_request_index = 0;

uint32_t lr_tick_count = 0;
uint8_t lr_nw_connect_mode;
uint8_t lr_nw_disconnect_mode;

uint8_t lr_nw_connect_timer_enable = 0;
uint8_t lr_nw_disconnect_timer_enable = 0;
uint8_t lr_nw_read_request_timer_enable = 0;

uint32_t lr_nw_connect_timer_count = 0;
uint32_t lr_nw_disconnect_timer_count = 0;
uint32_t lr_nw_read_request_timer_count = 0;


// Define device's class here
uint8_t NODE_DEVICE_CLASS[MAX_NODE_QUANTITY] = {0};
lora_node_t NODE_DEVICE[MAX_NODE_QUANTITY] = {0};
lora_node_t CONNECTED_NODE[MAX_NODE_QUANTITY] = {0};

TaskHandle_t connect_to_all_nodes_task_handle;
TaskHandle_t disconnect_to_all_nodes_task_handle;
TaskHandle_t send_read_request_to_all_nodes_task_handle;
TaskHandle_t handle_last_lora_send_packets_handle;
TaskHandle_t response_warning_nodes_task_handle;
TaskHandle_t send_data_to_pc_task_handle;

TimerHandle_t all_connect_timer;
TimerHandle_t all_disconnect_timer;
TimerHandle_t send_data_timer;

QueueHandle_t all_connect_queue;
QueueHandle_t all_disconnect_queue;
QueueHandle_t send_request_queue;
QueueHandle_t warning_queue;
QueueHandle_t pc_send_queue;

SemaphoreHandle_t spi_mutex;
SemaphoreHandle_t usb_mutex;
SemaphoreHandle_t last_lora_send_packet_counting_semaphore;
volatile uint8_t semaphore_count = 0;

lora_packet_t last_lora_send_packet_buffer[NUMBER_OF_LAST_LORA_SEND_PACKET];


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern lora_packet_t lora_receive_packet_buffer;
extern lora_packet_t lora_send_packet_buffer;

void lora_network_increase_tickcount() {lr_tick_count++;}

void lora_network_connect_to_all_nodes_task(void *param);
void lora_network_disconnect_to_all_nodes_task(void *param);
void lora_network_send_read_request_to_all_nodes_task(void *param);
void lora_network_handle_last_lora_send_packets_task(void *param);
void lora_network_response_warning_nodes_task(void *param);
void pc_user_send_data_to_pc_task(void *param);

void lora_network_timer_task(TimerHandle_t xTimer);
void lora_network_timer_connect_task(TimerHandle_t xTimer);
void lora_network_timer_disconnect_task(TimerHandle_t xTimer);
void lora_network_timer_read_request_task(TimerHandle_t xTimer);


void lora_network_send_request_stop_scheduler();
void lora_network_send_request_start_scheduler();
void lora_network_send_request_setup_scheduler();
void lora_network_send_request_reset_scheduler();

int lora_network_get_class_period(lora_node_t *lora_node){
	if(lora_node->class == DEVICE_CLASS_A)
		return DEVICE_CLASS_A_TIME_PERIOD;

	else if(lora_node->class == DEVICE_CLASS_B)
		return DEVICE_CLASS_B_TIME_PERIOD;

	else if(lora_node->class == DEVICE_CLASS_C)
		return DEVICE_CLASS_C_TIME_PERIOD;
	return -1;
}

uint8_t lora_network_connection_processing(){
	if(lr_nw_connect_mode != LORA_STATUS_IDLE || lr_nw_disconnect_mode != LORA_STATUS_IDLE){
		return 1;
	}
	return 0;
}

void lora_network_init(uint8_t node_quantity){
	bsp_lora_init();
	lr_nw_node_quantity = node_quantity;
	STM_LOG("node numbers: %d\n", lr_nw_node_quantity, 17);

	for(int i = 0; i < lr_nw_node_quantity; i++){
		NODE_DEVICE[i].id = i + 1;
		NODE_DEVICE[i].class = NODE_DEVICE_CLASS[i];
		NODE_DEVICE[i].connected = 0;
	}

	spi_mutex = xSemaphoreCreateMutex();
	usb_mutex = xSemaphoreCreateMutex();
	last_lora_send_packet_counting_semaphore = xSemaphoreCreateCounting(NUMBER_OF_LAST_LORA_SEND_PACKET, 0);

	xTaskCreate(lora_network_connect_to_all_nodes_task, "connect_all_nodes", 2048, NULL, 5, &connect_to_all_nodes_task_handle);
	xTaskCreate(lora_network_disconnect_to_all_nodes_task, "disconnect_all_nodes", 2048, NULL, 5, &disconnect_to_all_nodes_task_handle);
	xTaskCreate(lora_network_send_read_request_to_all_nodes_task, "send_request_all_nodes", 4096, NULL, 3, &send_read_request_to_all_nodes_task_handle);
	//xTaskCreate(lora_network_handle_last_lora_send_packets_task, "handle_last_packets", 512, NULL, 4, &handle_last_lora_send_packets_handle);
	xTaskCreate(lora_network_response_warning_nodes_task, "warning_response", 512, NULL, 6, &response_warning_nodes_task_handle);
	xTaskCreate(pc_user_send_data_to_pc_task, "pc_send_data", 512, NULL, 6, &send_data_to_pc_task_handle);

	all_connect_queue = xQueueCreate(3, sizeof(connection_task_notification_value_t));
	all_disconnect_queue = xQueueCreate(3, sizeof(connection_task_notification_value_t));
	send_request_queue = xQueueCreate(3, sizeof(connection_task_notification_value_t));
	warning_queue = xQueueCreate(3, sizeof(connection_task_notification_value_t));
	pc_send_queue = xQueueCreate(3, sizeof(pc_send_queue_t));


	lr_nw_connect_mode = LORA_STATUS_IDLE;
	lr_nw_disconnect_mode = LORA_STATUS_IDLE;
	HAL_TIM_Base_Start_IT(&htim4);
}

void lora_network_start_connect_timer(){
	lr_nw_connect_timer_enable = 1;
	lr_nw_connect_timer_count = 0;
}

void lora_network_stop_connect_timer(){
	lr_nw_connect_timer_enable = 0;
	lr_nw_connect_timer_count = 0;
}

void lora_network_start_disconnect_timer(){
	lr_nw_disconnect_timer_enable = 1;
	lr_nw_disconnect_timer_count = 0;
}

void lora_network_stop_disconnect_timer(){
	lr_nw_disconnect_timer_enable = 0;
	lr_nw_disconnect_timer_count = 0;
}

void lora_network_start_read_request_timer(){
	lr_nw_read_request_timer_enable = 1;
	lr_nw_read_request_timer_count = 0;
}

void lora_network_stop_read_request_timer(){
	lr_nw_read_request_timer_enable = 0;
	lr_nw_read_request_timer_count = 0;
}

void pc_user_ack_response(){
	uint8_t buffer[2];
	buffer[0] = 0;
	buffer[1] = PC_CMD_ACK;
	CDC_Transmit_FS(buffer, 2);
}

void pc_user_ack_response_connected_node_quantity(){
	//xSemaphoreTake(usb_mutex, portMAX_DELAY);
	/*uint8_t buffer[20];
	buffer[0] = lr_nw_connected_node_quantity;
	buffer[1] = PC_CMD_RESPONSE_CONNECTED_NODE_QUANTITY;
	buffer[2] = lr_nw_connected_node_quantity;
	for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
		buffer[i + 3] = CONNECTED_NODE[i].id;
	}
	CDC_Transmit_FS(buffer, lr_nw_connected_node_quantity + 3);*/
	//xSemaphoreGive(usb_mutex);
	pc_send_queue_t NotificationValue;
	NotificationValue.NotificationValue = PC_SEND_CONNECTED_NODE_QUANTITY;
	xQueueSend(pc_send_queue, &NotificationValue, portMAX_DELAY);
}

void pc_user_send_node_data_to_pc(uint8_t len, uint8_t node_id, uint8_t *data){
	/*xSemaphoreTake(usb_mutex, portMAX_DELAY);
	uint8_t buffer[35];
	buffer[0] = len;
	buffer[1] = PC_CMD_SEND_NODE_DATA;
	buffer[2] = node_id;

	memcpy((buffer + 3), data, len);
	CDC_Transmit_FS(buffer, len + 3);
	xSemaphoreGive(usb_mutex);*/
	pc_send_queue_t NotificationValue;
	NotificationValue.NotificationValue = PC_SEND_NODE_DATA;
	NotificationValue.usb_data.node_id = node_id;
	NotificationValue.usb_data.len = len;
	memcpy(NotificationValue.usb_data.data, data, len);
	xQueueSend(pc_send_queue, &NotificationValue, portMAX_DELAY);
}

void pc_user_send_data_to_pc_task(void *param){
	pc_send_queue_t NotificationValue;
	while(1){
		if(xQueueReceive(pc_send_queue, &NotificationValue, portMAX_DELAY)){
			STM_LOG("send data to pc\n", 0, 16);
			if(NotificationValue.NotificationValue == PC_SEND_CONNECTED_NODE_QUANTITY){
				uint8_t buffer[20];
				buffer[0] = lr_nw_connected_node_quantity;
				buffer[1] = PC_CMD_RESPONSE_CONNECTED_NODE_QUANTITY;
				buffer[2] = lr_nw_connected_node_quantity;
				for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
					buffer[i + 3] = CONNECTED_NODE[i].id;
				}
				CDC_Transmit_FS(buffer, lr_nw_connected_node_quantity + 3);
			}
			else if(NotificationValue.NotificationValue == PC_SEND_NODE_DATA){
				uint8_t buffer[45];
				buffer[0] = NotificationValue.usb_data.len;
				buffer[1] = PC_CMD_SEND_NODE_DATA;
				buffer[2] = NotificationValue.usb_data.node_id;

				memcpy(&buffer[3], NotificationValue.usb_data.data, NotificationValue.usb_data.len);
				CDC_Transmit_FS(buffer, NotificationValue.usb_data.len + 3);
				STM_LOG("Motor1 status: %2d\n", buffer[12], 17);
			}
		}
	}
}

void lora_network_disconnect_to_all_nodes_task(void *param){
	connection_task_notification_value_t NotificationValue;
	//all_disconnect_timer = xTimerCreate("disconnect_all", pdMS_TO_TICKS(1000), pdTRUE, (void *)SW_TIMER_DISCONNECT_ALL, lora_network_timer_disconnect_task);
	//vTimerSetTimerID(all_disconnect_timer, (void *)SW_TIMER_DISCONNECT_ALL);

	while(1){
		if(xQueueReceive(all_disconnect_queue, &NotificationValue, portMAX_DELAY)){
			/* get spi key and enter to safe mode to avoid the situation when lora's data's conflicted */
			xSemaphoreTake(spi_mutex, portMAX_DELAY);
			bsp_lora_enter_safe_mode();
			bsp_pc_user_enter_safe_mode();
			lora_network_send_request_stop_scheduler();

			while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			while(bsp_lora_check_cad() == 1);
			bsp_lora_set_receive_mode();
			if(NotificationValue.cmd == STOP_ALL){
				while(NODE_DEVICE[lr_nw_node_index1].connected == 0 && lr_nw_node_index1 < lr_nw_node_quantity){
					lr_nw_node_index1++;
				}
				if(lr_nw_node_index1 < lr_nw_node_quantity && NODE_DEVICE[lr_nw_node_index1].connected == 1){
					HAL_UART_Transmit(&huart2, "Start disconnect\n", 18, 2000);
					bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT, 0, 0, 0, 3);
					//xTimerStart(all_disconnect_timer, 0);
					lora_network_start_disconnect_timer();
				}
			}


			/* disconnect to specific device here */
			else if(NotificationValue.cmd == STOP_SPECIFIC){
				HAL_UART_Transmit(&huart2, "Stop specific\n", 16, 2000);
				for(uint8_t i = 0; i < lr_nw_node_quantity; i++){
					if(NODE_DEVICE[i].id == NotificationValue.node_id){
						lr_nw_node_index1 = i;
						break;
					}
				}
				if(NODE_DEVICE[lr_nw_node_index1].connected == 1){
					bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT, 0, 0, 0, 3);
					//xTimerStart(all_disconnect_timer, 0);
					lora_network_start_disconnect_timer();
				}
				else{
					HAL_UART_Transmit(&huart2, "Node's disconnected\n", 20, 2000);
					lr_nw_node_index1 = 0;

					taskENTER_CRITICAL();
					lr_nw_disconnect_mode = LORA_STATUS_IDLE;
					taskEXIT_CRITICAL();
				}
			}

			/* when receive ack cmd from the node that the gw is requesting to disconnect to */
			else if(NotificationValue.cmd == ACK){
				//xQueueReset(all_connect_queue);
				uint8_t buf[30];
				//xTimerStop(all_disconnect_timer, 0);
				sprintf(buf, "Disconnect to node %d\n", lr_nw_node_index1 + 1);
				HAL_UART_Transmit(&huart2, buf, strlen(buf), 2000);

				int packet_index = bsp_lora_check_cmd_in_node_send_packets(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT);
				if(packet_index != -1){
					bsp_lora_remove_packet_from_node_send_packets(&NODE_DEVICE[lr_nw_node_index1], packet_index);
				}

				if(lr_nw_disconnect_mode == LORA_DISCONNECT_MODE_ALL){
					do{
						lr_nw_node_index1++;
					}while(NODE_DEVICE[lr_nw_node_index1].connected == 0 && lr_nw_node_index1 < lr_nw_node_quantity);

					if(lr_nw_node_index1 < lr_nw_node_quantity && NODE_DEVICE[lr_nw_node_index1].connected == 1){
						HAL_UART_Transmit(&huart2, "Disconnect sucessfully\n", 23, 2000);
						bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT, 0, 0, 0, 3);
						//xTimerStart(all_disconnect_timer, 0);
						lora_network_start_disconnect_timer();
					}
				}
				else if(lr_nw_disconnect_mode == LORA_DISCONNECT_MODE_SPECIFIC){
					lr_nw_node_index1 = 0;
					taskENTER_CRITICAL();
					lr_nw_disconnect_mode = LORA_STATUS_IDLE;
					taskEXIT_CRITICAL();
				}
			}

			/* receive nak or timeout */
			else if(NotificationValue.cmd == NAK || NotificationValue.cmd == TIMEOUT){
				//xTimerStop(all_disconnect_timer, 0);
				STM_LOG("Timeout n%2d\n", (lr_nw_node_index1 + 1), 11);
				int packet_index = bsp_lora_check_cmd_in_node_send_packets(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT);
				if(packet_index != -1){
					if(NODE_DEVICE[lr_nw_node_index1].last_lora_send_packet[packet_index].ttl > 0 && NODE_DEVICE[lr_nw_node_index1].last_lora_send_packet[packet_index].responsed == 0){
						STM_LOG("Disconnect node %3d tout\n", lr_nw_node_index1 + 1, 26);
						NODE_DEVICE[lr_nw_node_index1].error = 1;
						bsp_lora_resend_packet(&NODE_DEVICE[lr_nw_node_index1].last_lora_send_packet[packet_index]);
						//xTimerStart(all_disconnect_timer, 0);
						lora_network_start_disconnect_timer();

					}
					else{
						STM_LOG("Can't disconnect node %3d\n", lr_nw_node_index1 + 1, 27);
						NODE_DEVICE[lr_nw_node_index1].error = 2;
						bsp_lora_remove_packet_from_node_send_packets(&NODE_DEVICE[lr_nw_node_index1], packet_index);

						if(lr_nw_disconnect_mode == LORA_DISCONNECT_MODE_ALL){
							do{
								lr_nw_node_index1++;
							}while(NODE_DEVICE[lr_nw_node_index1].connected == 0 && lr_nw_node_index1 < lr_nw_node_quantity);
							if(lr_nw_node_index1 < lr_nw_node_quantity && NODE_DEVICE[lr_nw_node_index1].connected == 1){
								bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index1], LORA_CMD_DISCONNECT, 0, NULL, 0, 3);
								//xTimerStart(all_disconnect_timer, 0);
								lora_network_start_disconnect_timer();
							}
						}
						else if(lr_nw_disconnect_mode == LORA_DISCONNECT_MODE_SPECIFIC){
							lr_nw_node_index1 = 0;
							taskENTER_CRITICAL();
							lr_nw_disconnect_mode = LORA_STATUS_IDLE;
							taskEXIT_CRITICAL();
						}
					}
				}

			}


			if(lr_nw_node_index1 >= lr_nw_node_quantity){

				HAL_UART_Transmit(&huart2, "Empty nodes\n", 12, 2000);
				lr_nw_node_index1 = 0;

				taskENTER_CRITICAL();
				lr_nw_disconnect_mode = LORA_STATUS_IDLE;
				taskEXIT_CRITICAL();

			}

			bsp_pc_user_enter_safe_mode();

			//STM_LOG("node device: %d\n", lr_nw_node_index1 + 1, 14);
			if(lr_nw_disconnect_mode == LORA_STATUS_IDLE){
				xQueueReset(send_request_queue);
				xQueueReset(all_disconnect_queue);
				lora_network_send_request_setup_scheduler();
				pc_user_ack_response_connected_node_quantity();

			}

			/* exit safe mode */
			bsp_lora_exit_safe_mode();
			while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			xSemaphoreGive(spi_mutex);
		}

	}
	//vTaskDelete(NULL);
}

void lora_network_connect_to_all_nodes_task(void *param){
	connection_task_notification_value_t NotificationValue;
	//all_connect_timer = xTimerCreate("connect_all", pdMS_TO_TICKS(1000), pdTRUE, 0, lora_network_timer_connect_task);
	//vTimerSetTimerID(all_connect_timer, (void *)SW_TIMER_CONNECT_ALL);

	while(1){
		if(xQueueReceive(all_connect_queue, &NotificationValue, portMAX_DELAY)){
			/* get spi key and enter to safe mode to avoid the situation when lora's data's conflicted */
			xSemaphoreTake(spi_mutex, portMAX_DELAY);
			bsp_lora_enter_safe_mode();
			bsp_pc_user_enter_safe_mode();
			lora_network_send_request_stop_scheduler();

			while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			while(bsp_lora_check_cad() == 1);
			bsp_lora_set_receive_mode();
			if(NotificationValue.cmd == START_ALL){
				while(NODE_DEVICE[lr_nw_node_index].connected == 1 && lr_nw_node_index < lr_nw_node_quantity){
					lr_nw_node_index++;
				}
				if(NODE_DEVICE[lr_nw_node_index].connected == 0 && lr_nw_node_index < lr_nw_node_quantity){
					HAL_UART_Transmit(&huart2, "Start connect\n", 15, 2000);
					bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT, 0, NULL, 0, 3);
					//xTimerStart(all_connect_timer, 10);
					lora_network_start_connect_timer();
				}
			}
			/* connect to specific device here */
			else if(NotificationValue.cmd == START_SPECIFIC){
				HAL_UART_Transmit(&huart2, "Start specific\n", 16, 2000);
				for(int i = 0; i < lr_nw_node_quantity; i++){
					if(NODE_DEVICE[i].id == NotificationValue.node_id){
						lr_nw_node_index = i;
						break;
					}
				}
				if(NODE_DEVICE[lr_nw_node_index].connected == 0){
					bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT, 0, NULL, 0, 3);
					//xTimerStart(all_connect_timer, 0);
					lora_network_start_connect_timer();
				}
				else{
					HAL_UART_Transmit(&huart2, "Node's connected\n", 17, 2000);
					lr_nw_node_index = 0;

					taskENTER_CRITICAL();
					lr_nw_connect_mode = LORA_STATUS_IDLE;
					taskEXIT_CRITICAL();
				}
			}

			/* when receive ack cmd from the node that the gw is requesting to connect to */
			else if(NotificationValue.cmd == ACK){
				//xTimerStop(all_connect_timer, 0);
				STM_LOG("Connect to node %2d\n", lr_nw_node_index + 1, 20);
				int packet_index = bsp_lora_check_cmd_in_node_send_packets(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT);
				if(packet_index != -1){
					bsp_lora_remove_packet_from_node_send_packets(&NODE_DEVICE[lr_nw_node_index], packet_index);
				}

				if(lr_nw_connect_mode == LORA_CONNECT_MODE_ALL){
					do{
						lr_nw_node_index++;
					}while(NODE_DEVICE[lr_nw_node_index].connected == 1 && lr_nw_node_index < lr_nw_node_quantity);

					if(lr_nw_node_index < lr_nw_node_quantity && NODE_DEVICE[lr_nw_node_index].connected == 0){
						HAL_UART_Transmit(&huart2, "Connect sucessfully\n", 20, 2000);
						bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT, 0, NULL, 0, 3);
						//xTimerStart(all_connect_timer, 0);
						lora_network_start_connect_timer();
					}
				}
				else if(lr_nw_connect_mode == LORA_CONNECT_MODE_SPECIFIC){
					lr_nw_node_index = 0;

					taskENTER_CRITICAL();
					lr_nw_connect_mode = LORA_STATUS_IDLE;
					taskEXIT_CRITICAL();
				}

			}

			/* receive nak or timeout */
			else if(NotificationValue.cmd == NAK || NotificationValue.cmd == TIMEOUT){
				//xTimerStop(all_connect_timer, 0);
				HAL_UART_Transmit(&huart2, "Connect timeout\n", 16, 2000);
				STM_LOG("Connect timeout n%2d\n", lr_nw_node_index, 20);
				int packet_index = bsp_lora_check_cmd_in_node_send_packets(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT);
				if(packet_index != -1){
					if(NODE_DEVICE[lr_nw_node_index].last_lora_send_packet[packet_index].ttl > 0){
						NODE_DEVICE[lr_nw_node_index].error = 1;
						bsp_lora_resend_packet(&NODE_DEVICE[lr_nw_node_index].last_lora_send_packet[packet_index]);

						//xTimerStart(all_connect_timer, 0);
						lora_network_start_connect_timer();
					}
					else{
						STM_LOG("Can't connect to node %3d\n", lr_nw_node_index + 1, 27);
						NODE_DEVICE[lr_nw_node_index].error = 2;
						bsp_lora_remove_packet_from_node_send_packets(&NODE_DEVICE[lr_nw_node_index], packet_index);

						if(lr_nw_connect_mode == LORA_CONNECT_MODE_ALL){
							do{
								lr_nw_node_index++;
							}while(NODE_DEVICE[lr_nw_node_index].connected == 1 && lr_nw_node_index < lr_nw_node_quantity);
							if(lr_nw_node_index < lr_nw_node_quantity && NODE_DEVICE[lr_nw_node_index].connected == 0){
								bsp_lora_send_packet_to_node(&NODE_DEVICE[lr_nw_node_index], LORA_CMD_CONNECT, 0, NULL, 0, 3);
								//xTimerStart(all_connect_timer, 0);
								lora_network_start_connect_timer();
							}
						}
						else if(lr_nw_connect_mode == LORA_CONNECT_MODE_SPECIFIC){
							lr_nw_node_index = 0;
							taskENTER_CRITICAL();
							lr_nw_connect_mode = LORA_STATUS_IDLE;
							taskEXIT_CRITICAL();
						}
					}
				}

			}


			if(lr_nw_node_index >= lr_nw_node_quantity){

				HAL_UART_Transmit(&huart2, "Full nodes\n", 11, 2000);
				lr_nw_node_index = 0;

				taskENTER_CRITICAL();
				lr_nw_connect_mode = LORA_STATUS_IDLE;
				taskEXIT_CRITICAL();

			}

			bsp_pc_user_exit_safe_mode();

			if(lr_nw_connect_mode == LORA_STATUS_IDLE){
				lora_network_send_request_setup_scheduler();
				pc_user_ack_response_connected_node_quantity();
			}
			/* exit safe mode */
			bsp_lora_exit_safe_mode();
			while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			xSemaphoreGive(spi_mutex);
		}

	}
}

void lora_network_send_read_request_to_all_nodes_task(void *param){
	connection_task_notification_value_t NotificationValue;
	//send_data_timer = xTimerCreate("send_data", pdMS_TO_TICKS(1000), pdTRUE, 0, lora_network_timer_read_request_task);
	//vTimerSetTimerID(send_data_timer, (void *)SW_TIMER_SEND_REQUEST);
	while(1){
		while(lora_network_connection_processing());
		if(xQueueReceive(send_request_queue, &NotificationValue, portMAX_DELAY)){
			/* get spi key and enter to safe mode to avoid the situation when lora's data's conflicted */
			xSemaphoreTake(spi_mutex, portMAX_DELAY);
			bsp_lora_enter_safe_mode();
			STM_LOG("Send read request task\n", 0, 23);



			while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			while(bsp_lora_check_cad() == 1);
			bsp_lora_set_receive_mode();

			STM_LOG("Send notice v: %2d\n", NotificationValue.cmd, 19);

			/* start send request */
			if(NotificationValue.cmd == START_SEND_REQUEST){
				if(CONNECTED_NODE[lr_nw_send_request_index].connected == 1 && CONNECTED_NODE[lr_nw_send_request_index].id != 0){
					STM_LOG("Send request n%2d\n", lr_nw_send_request_index, 17);
					STM_LOG("Node's id: %2d\n", CONNECTED_NODE[lr_nw_send_request_index].id, 14);
					bsp_lora_send_packet_to_node(&CONNECTED_NODE[lr_nw_send_request_index], LORA_CMD_READ_DATA, 0, NULL, 0, 3);
					//xTimerStart(send_data_timer, 0);
					lora_network_start_read_request_timer();
				}
				else{
					lr_nw_send_request_index = 0;
				}
			}

			/* receive ack for the packet have sent */
			else if(NotificationValue.cmd == ACK){
				//xTimerStop(send_data_timer, 0);


				STM_LOG("Ack send request n%2d\n", lr_nw_send_request_index, 22);
				if(CONNECTED_NODE[lr_nw_send_request_index].connected == 1 && CONNECTED_NODE[lr_nw_send_request_index].id != 0){
					int packet_index = bsp_lora_check_cmd_in_node_send_packets(&CONNECTED_NODE[lr_nw_send_request_index], LORA_CMD_READ_DATA);
					if(packet_index != -1){
						STM_LOG("Remove packet with pid: %3d\n", CONNECTED_NODE[lr_nw_send_request_index].last_lora_send_packet[packet_index].packet_id, 28);
						bsp_lora_remove_packet_from_node_send_packets(&CONNECTED_NODE[lr_nw_send_request_index], packet_index);
					}
					lr_nw_send_request_index++;
				}
				else{
					lr_nw_send_request_index = 0;
				}
			}

			/* timeout or receive nack for the packet have sent */
			else if(NotificationValue.cmd == NAK || NotificationValue.cmd == TIMEOUT){
				STM_LOG("Send request timeout or nak n%2d\n", lr_nw_send_request_index, 32);
				int packet_index = bsp_lora_check_cmd_in_node_send_packets(&CONNECTED_NODE[lr_nw_send_request_index], LORA_CMD_READ_DATA);
				//xTimerStop(send_data_timer, 0);
				if(packet_index != -1){
					if(CONNECTED_NODE[lr_nw_send_request_index].last_lora_send_packet[packet_index].ttl > 0 && CONNECTED_NODE[lr_nw_send_request_index].last_lora_send_packet[packet_index].responsed == 0){
						if(CONNECTED_NODE[lr_nw_send_request_index].connected == 1 && CONNECTED_NODE[lr_nw_send_request_index].id != 0){
							STM_LOG("Resend dcn packet n%2d\n", lr_nw_send_request_index, 22);
							CONNECTED_NODE[lr_nw_send_request_index].error = 1;
							bsp_lora_resend_packet(&CONNECTED_NODE[lr_nw_send_request_index].last_lora_send_packet);
							//xTimerStart(send_data_timer, 0);
							lora_network_start_read_request_timer();
						}
					}
					else{
						if(CONNECTED_NODE[lr_nw_send_request_index].connected == 1 && CONNECTED_NODE[lr_nw_send_request_index].id != 0){
							CONNECTED_NODE[lr_nw_send_request_index].error = 2;
							bsp_lora_remove_packet_from_node_send_packets(&CONNECTED_NODE[lr_nw_send_request_index], packet_index);
							STM_LOG("Error n%2d\n", lr_nw_send_request_index, 10);
							lr_nw_send_request_index++;
						}
					}
				}
				else{
					//xTimerStop(send_data_timer, 0);
				}


			}
			if(lr_nw_send_request_index >= lr_nw_connected_node_quantity)
				lr_nw_send_request_index = 0;

			/* exit safe mode */
			while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			bsp_lora_exit_safe_mode();
			//NVIC_EnableIRQ(OTG_FS_IRQn);
			xSemaphoreGive(spi_mutex);
		}

	}
}

void lora_network_send_request_stop_scheduler(){
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	HAL_TIM_Base_Stop_IT(&htim2);
}

void lora_network_send_request_start_scheduler(){
	//TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	//TIM2->CR1 |= TIM_CR1_CEN;
	HAL_TIM_Base_Start_IT(&htim2);
}

void lora_network_send_request_reset_scheduler(){
	lora_network_send_request_stop_scheduler();
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
	lora_network_send_request_start_scheduler();
}


void lora_network_send_request_setup_scheduler(){
	lora_network_send_request_stop_scheduler();
	if(lr_nw_connected_node_quantity > 0){
		HAL_UART_Transmit(&huart2, "Scheduler setup\n", 16, 2000);
		STM_LOG("Nqtt: %2d\n", lr_nw_connected_node_quantity, 10);

		lr_nw_last_connected_node_quantity = lr_nw_connected_node_quantity;
		uint32_t time_interval = SEND_REQUEST_TIME_PERIOD / lr_nw_connected_node_quantity;
		TIM2->CR1 &= ~TIM_CR1_CEN;
		TIM2->CNT = 0;
		TIM2->ARR = time_interval - 1;

		if(semaphore_count == 0){
			connection_task_notification_value_t NotificationValue;
			NotificationValue.cmd = START_SEND_REQUEST;
			NotificationValue.node_id = 0;
			xQueueSend(send_request_queue, &NotificationValue, portMAX_DELAY);
			lora_network_send_request_start_scheduler();
		}
		lora_network_send_request_start_scheduler();
	}

	uart_data_frame_t uart_data_buffer = {0};
	uart_data_buffer.len = lr_nw_connected_node_quantity + 2;
	uart_data_buffer.cmd = UART_CMD_INITIALISE_NODE_QUANTITY;
	uart_data_buffer.node_id = lr_nw_connected_node_quantity;

	for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
		uart_data_buffer.raw_data[i] = CONNECTED_NODE[i].id;
	}

	uint8_t *packet_bufer = (uint8_t *)(&uart_data_buffer);
	HAL_UART_Transmit(&huart1, packet_bufer, sizeof(uart_data_frame_t), 2000);
	//lora_network_send_request_start_scheduler();
}

void lora_network_send_request_scheduler(){
	if(semaphore_count == 0 && lr_nw_connected_node_quantity > 0){
		STM_LOG("Scheduler call back\n", 0, 19);

		//NVIC_DisableIRQ(OTG_FS_IRQn);
		BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
		connection_task_notification_value_t NotificationValue;
		NotificationValue.cmd = START_SEND_REQUEST;
		NotificationValue.node_id = 0;
		//xTimerStopFromISR(send_data_timer, &pxHigherPriorityTaskWoken);
		xQueueSendFromISR(send_request_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}

}


/* this function is used to send remain lora packets that haven't sent */
void lora_network_handle_last_lora_send_packets_task(void *param){
	connection_task_notification_value_t NotificationValue;
	while(1){
		xSemaphoreTake(last_lora_send_packet_counting_semaphore, portMAX_DELAY);
		xSemaphoreTake(spi_mutex, portMAX_DELAY);
		bsp_lora_enter_safe_mode();

		STM_LOG("Handle last packet\n", 0, 19);
		STM_LOG("Last packet cmd: %2d\n", last_lora_send_packet_buffer[0].cmd, 20);

		/* if last lora send packet is a packet contain CONNECT command */
		if(last_lora_send_packet_buffer[0].cmd == LORA_CMD_CONNECT){
			STM_LOG("Last packet is connect packet\n", 0, 29);
			NotificationValue.cmd = NAK;
			xQueueSend(all_connect_queue, &NotificationValue, portMAX_DELAY);
		}

		/* if last lora send packet is a packet contain DISCONNECT command */
		else if(last_lora_send_packet_buffer[0].cmd == LORA_CMD_DISCONNECT){
			STM_LOG("Last packet is disconnect packet\n", 0, 32);
			NotificationValue.cmd = NAK;
			xQueueSend(all_disconnect_queue, &NotificationValue, portMAX_DELAY);
		}

		/* if last lora send packet is a packet contain READ_DATA command */
		else if(last_lora_send_packet_buffer[0].cmd == LORA_CMD_READ_DATA){
			lora_network_send_request_reset_scheduler();
			STM_LOG("Last packet is send request packet\n", 0, 35);
			NotificationValue.cmd = NAK;
			NotificationValue.node_id = 0;
			xQueueSend(send_request_queue, &NotificationValue, portMAX_DELAY);
		}

		for(uint8_t i = 0; i < semaphore_count - 1; i++){
			memcpy(&last_lora_send_packet_buffer[i], &last_lora_send_packet_buffer[i + 1], sizeof(lora_packet_t));
		}
		memset(&last_lora_send_packet_buffer[semaphore_count - 1], 0, sizeof(lora_packet_t));
		semaphore_count--;

		bsp_lora_exit_safe_mode();
		xSemaphoreGive(spi_mutex);
	}
}


/* this fuction is used to response the warning packets from the nodes */
void lora_network_response_warning_nodes_task(void *param){
	connection_task_notification_value_t NotificationValue;
	while(1){
		if(xQueueReceive(warning_queue, &NotificationValue, portMAX_DELAY)){
			xSemaphoreTake(spi_mutex, portMAX_DELAY);
			bsp_lora_enter_safe_mode();
			lora_network_send_request_stop_scheduler();

			while(HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			while(bsp_lora_check_cad() == 1);
			bsp_lora_set_receive_mode();

			if(NotificationValue.cmd == START_RESPONSE_WARNING){
				uint8_t lora_node_id = NotificationValue.node_id;
				lora_node_t *lora_node = NULL;
				for(uint8_t node_index = 0; node_index < lr_nw_connected_node_quantity; node_index++){
					if(CONNECTED_NODE[node_index].id == lora_node_id){
						lora_node = &CONNECTED_NODE[node_index];
						break;
					}
				}

				if(lora_node != NULL){
					int packet_index = bsp_lora_check_cmd_in_node_receive_packets(lora_node, LORA_CMD_WARNING);

					if(packet_index != -1){
						uart_data_frame_t uart_data_buffer = {0};
						uart_data_buffer.len = 3;
						uart_data_buffer.cmd = UART_CMD_HANDLE_WARNING_NODE;
						uart_data_buffer.node_id = lora_node->id;
						uart_data_buffer.raw_data[0] = lora_node->connected;
						uart_data_buffer.raw_data[1] = lora_node->error;
						// the first item of the payload contain information about the machine and remaining elements contain information about different things such as relay or contactor...
						// = 0: not working; = 1: working; = 2: error
						uart_data_buffer.raw_data[2] = lora_node->last_lora_receive_packet[packet_index].payload[0];
						uint8_t *packet_bufer = (uint8_t *)(&uart_data_buffer);
						HAL_UART_Transmit(&huart1, packet_bufer, sizeof(uart_data_frame_t), 2000);
						pc_user_send_node_data_to_pc(uart_data_buffer.len, uart_data_buffer.node_id, uart_data_buffer.raw_data);

						STM_LOG("First item of warning data: %2d\n", uart_data_buffer.raw_data[2], 32);

						uint8_t responsed_byte = lora_node->last_lora_receive_packet[packet_index].packet_id;

						//memset(&lora_node->last_lora_receive_packet[packet_index], 0, sizeof(lora_packet_t));
						bsp_lora_remove_packet_from_node_receive_packets(lora_node, packet_index);

						lora_send_packet_buffer.responsed = responsed_byte;
						bsp_lora_send_packet_to_node(lora_node, LORA_CMD_ACK, 0, NULL, 0, 3);
					}
				}
			}

			else if(NotificationValue.cmd == START_HANDLE_ACK_PACKET){
				uint8_t lora_node_id = NotificationValue.node_id;
				lora_node_t *lora_node = NULL;

				//HAL_UART_Transmit(&huart2, "ack packet debug 1\n", 19, 2000);
				for(uint8_t node_index = 0; node_index < lr_nw_connected_node_quantity; node_index++){
					if(CONNECTED_NODE[node_index].id == lora_node_id){
						lora_node = &CONNECTED_NODE[node_index];
						break;
					}
				}

				if(lora_node != NULL){
					//HAL_UART_Transmit(&huart2, "ack packet debug 2\n", 19, 2000);

					int packet_index = bsp_lora_get_id_in_node_receive_packets(lora_node, NotificationValue.packet_id);
					if(packet_index != -1){

						//HAL_UART_Transmit(&huart2, "ack packet debug 3\n", 19, 2000);

						uart_data_frame_t uart_data_buffer = {0};
						uart_data_buffer.len = 40;
						uart_data_buffer.cmd = UART_CMD_HANDLE_DATA_READ_FROM_NODE;
						uart_data_buffer.node_id = lora_node->id;
						uart_data_buffer.raw_data[0] = lora_node->connected;
						uart_data_buffer.raw_data[1] = lora_node->error;
						// the first item of the payload contain information about the machine and remaining elements contain information about different things such as relay or contactor...
						// = 0: not working; = 1: working; = 2: error
						uart_data_buffer.raw_data[2] = lora_node->last_lora_receive_packet[packet_index].payload[0];
						memcpy(&uart_data_buffer.raw_data[8], lora_node->last_lora_receive_packet[packet_index].payload, MAX_PAYLOAD_LENGTH);
						uint8_t *packet_bufer = (uint8_t *)(&uart_data_buffer);

						uint8_t debug_buffer[35];
						sprintf(debug_buffer, "ack buffer of msg pid:%2d,%2d,%3d\n", lora_node->last_lora_receive_packet[packet_index].payload[0], lora_node->last_lora_receive_packet[packet_index].payload[1], lora_node->last_lora_receive_packet[packet_index].responsed);
						HAL_UART_Transmit(&huart2, debug_buffer, sizeof(debug_buffer), 2000);

						HAL_UART_Transmit(&huart1, packet_bufer, sizeof(uart_data_frame_t), 2000);
						pc_user_send_node_data_to_pc(uart_data_buffer.len, uart_data_buffer.node_id, uart_data_buffer.raw_data);



						NotificationValue.cmd = ACK;
						NotificationValue.node_id = lora_node->id;
						NotificationValue.packet_id = lora_node->last_lora_receive_packet[packet_index].responsed;

						HAL_UART_Transmit(&huart2, "Notify\n", 7, 2000);
						if(uxQueueMessagesWaiting(send_request_queue) < 3)
							xQueueSend(send_request_queue, &NotificationValue, portMAX_DELAY);

						bsp_lora_remove_packet_from_node_receive_packets(lora_node, packet_index);
					}
				}
			}


			while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);
			lora_network_send_request_start_scheduler();

			STM_LOG("Quit send request\n", 0, 18);
			xSemaphoreGive(spi_mutex);
			bsp_lora_exit_safe_mode();
		}


	}
}


void lora_network_timer_connect_task(TimerHandle_t xTimer){
	/*uint8_t sw_timer_id = (uint8_t) pvTimerGetTimerID(xTimer);
	if(sw_timer_id == SW_TIMER_CONNECT_ALL){*/
	connection_task_notification_value_t NotificationValue;
	NotificationValue.cmd = TIMEOUT;
	NotificationValue.node_id = 0;
	HAL_UART_Transmit(&huart2, "Connect timeout\n", 15, 2000);
	//xTimerReset(all_connect_timer, portMAX_DELAY);
	xTimerStop(all_connect_timer, 0);
	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSend(all_connect_queue, &NotificationValue, portMAX_DELAY);
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	/*}
	else if(sw_timer_id == SW_TIMER_DISCONNECT_ALL){
		connection_task_notification_value_t NotificationValue;
		NotificationValue.cmd = TIMEOUT;
		NotificationValue.node_id = 0;
		HAL_UART_Transmit(&huart2, "Disconnect timeout\n", 18, 2000);
		xTimerStop(xTimer, 10);
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSend(all_disconnect_queue, &NotificationValue, portMAX_DELAY);
		xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else if(sw_timer_id == SW_TIMER_SEND_REQUEST){
		connection_task_notification_value_t NotificationValue;
		NotificationValue.cmd = TIMEOUT;
		NotificationValue.node_id = 0;
		HAL_UART_Transmit(&huart2, "Send request timeout\n", 21, 2000);
		xTimerStop(xTimer, 10);
		//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xQueueSend(send_request_queue, &NotificationValue, portMAX_DELAY);
		//xQueueSendFromISR(send_request_queue, &NotificationValue, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}*/
}

void lora_network_timer_disconnect_task(TimerHandle_t xTimer){
	connection_task_notification_value_t NotificationValue;
	NotificationValue.cmd = TIMEOUT;
	NotificationValue.node_id = 0;
	HAL_UART_Transmit(&huart2, "Disconnect timeout\n", 18, 2000);
	//xTimerReset(all_disconnect_timer, portMAX_DELAY);
	xTimerStop(all_disconnect_timer, 0);
	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSend(all_disconnect_queue, &NotificationValue, portMAX_DELAY);
	/*xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);*/
}

void lora_network_timer_read_request_task(TimerHandle_t xTimer){
	connection_task_notification_value_t NotificationValue;
	NotificationValue.cmd = TIMEOUT;
	NotificationValue.node_id = 0;
	HAL_UART_Transmit(&huart2, "Send request timeout\n", 21, 2000);
	//xTimerReset(send_data_timer, portMAX_DELAY);
	xTimerStop(send_data_timer, 0);
	//BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSend(send_request_queue, &NotificationValue, portMAX_DELAY);
	//xQueueSendFromISR(send_request_queue, &NotificationValue, &xHigherPriorityTaskWoken);
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/* if the gw is sending a packet through lora but not received ack packet yet, and at the same time it receive cmd from
 * user or receive warning or ack packet from other nodes, it will save this current packet and process incoming packet */
void lora_network_save_current_work(){
	if(lora_send_packet_buffer.destination_id != 0){
		if(lora_send_packet_buffer.responsed == 0){
			lora_node_t *lora_node;

			if(lora_send_packet_buffer.cmd == LORA_CMD_CONNECT || lora_send_packet_buffer.cmd == LORA_CMD_DISCONNECT){
				for(uint8_t i = 0; i < lr_nw_node_quantity; i++){
					if(NODE_DEVICE[i].id == lora_send_packet_buffer.destination_id){
						lora_node = &NODE_DEVICE[i];
						break;
					}
				}
			}
			else{
				for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
					if(CONNECTED_NODE[i].id == lora_send_packet_buffer.destination_id){
						lora_node = &CONNECTED_NODE[i];
						break;
					}
				}
			}

			/* if time to live byte of packet is still > 0 */
			if(lora_send_packet_buffer.ttl > 0){
				STM_LOG("save curent task1\n", 0, 18);
				xSemaphoreGiveFromISR(last_lora_send_packet_counting_semaphore, NULL);
				semaphore_count = (semaphore_count < NUMBER_OF_LAST_LORA_SEND_PACKET) ? semaphore_count + 1 : NUMBER_OF_LAST_LORA_SEND_PACKET;
				STM_LOG("smph c: %2d\n", semaphore_count, 11);
				memcpy(&last_lora_send_packet_buffer[semaphore_count - 1], &lora_send_packet_buffer, sizeof(lora_packet_t));


				if(lora_send_packet_buffer.cmd == LORA_CMD_CONNECT){
					/*xTimerStopFromISR(all_connect_timer, NULL);
					xTimerStopFromISR(all_disconnect_timer, NULL);
					xTimerStopFromISR(send_data_timer, NULL);*/
					lora_network_stop_connect_timer();
				}

				else if(lora_send_packet_buffer.cmd == LORA_CMD_DISCONNECT){
					/*xTimerStopFromISR(all_connect_timer, NULL);
					xTimerStopFromISR(all_disconnect_timer, NULL);
					xTimerStopFromISR(send_data_timer, NULL);*/
					lora_network_stop_disconnect_timer();
				}

				else if(lora_send_packet_buffer.cmd == LORA_CMD_READ_DATA){
					STM_LOG("Save the task with id %3d\n", last_lora_send_packet_buffer[semaphore_count - 1].packet_id, 26);
					/*xTimerStopFromISR(all_connect_timer, NULL);
					xTimerStopFromISR(all_disconnect_timer, NULL);
					xTimerStopFromISR(send_data_timer, NULL);*/
					lora_network_stop_read_request_timer();
				}
			}

			/* if time to live of packet equal to 0 but this packet haven't exceeded the timeout that mean the timer is
			 * still counting */
			else{
				if(lora_send_packet_buffer.cmd == LORA_CMD_CONNECT){
					if(lora_node->error <= 1){
						xSemaphoreGiveFromISR(last_lora_send_packet_counting_semaphore, NULL);
						semaphore_count = (semaphore_count < NUMBER_OF_LAST_LORA_SEND_PACKET) ? semaphore_count + 1 : NUMBER_OF_LAST_LORA_SEND_PACKET;
						lora_send_packet_buffer.ttl++;
						int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_CONNECT, lora_send_packet_buffer.packet_id);
						if(packet_index != -1){
							lora_node->last_lora_send_packet[packet_index].ttl++;
						}
						memcpy(&last_lora_send_packet_buffer[semaphore_count - 1], &lora_send_packet_buffer, sizeof(lora_packet_t));
						//xTimerStopFromISR(all_connect_timer, NULL);
						lora_network_stop_connect_timer();
					}
				}

				else if(lora_send_packet_buffer.cmd == LORA_CMD_DISCONNECT){
					if(lora_node->error <= 1){
						xSemaphoreGiveFromISR(last_lora_send_packet_counting_semaphore, NULL);
						semaphore_count = (semaphore_count < NUMBER_OF_LAST_LORA_SEND_PACKET) ? semaphore_count + 1 : NUMBER_OF_LAST_LORA_SEND_PACKET;
						lora_send_packet_buffer.ttl++;
						int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_DISCONNECT, lora_send_packet_buffer.packet_id);
						if(packet_index != -1){
							lora_node->last_lora_send_packet[packet_index].ttl++;
						}
						memcpy(&last_lora_send_packet_buffer[semaphore_count - 1], &lora_send_packet_buffer, sizeof(lora_packet_t));
						//xTimerStopFromISR(all_disconnect_timer, NULL);
						lora_network_stop_disconnect_timer();
					}
				}

				else if(lora_send_packet_buffer.cmd == LORA_CMD_READ_DATA){
					if(lora_node->error <= 1){
						xSemaphoreGiveFromISR(last_lora_send_packet_counting_semaphore, NULL);
						semaphore_count = (semaphore_count < NUMBER_OF_LAST_LORA_SEND_PACKET) ? semaphore_count + 1 : NUMBER_OF_LAST_LORA_SEND_PACKET;
						lora_send_packet_buffer.ttl++;
						int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_READ_DATA, lora_send_packet_buffer.packet_id);
						if(packet_index != -1){
							lora_node->last_lora_send_packet[packet_index].ttl++;
						}
						memcpy(&last_lora_send_packet_buffer[semaphore_count - 1], &lora_send_packet_buffer, sizeof(lora_packet_t));
						//xTimerStopFromISR(send_data_timer, NULL);
						lora_network_stop_read_request_timer();
					}
				}
			}
		}
	}
}



/*void pc_user_ack_response_step2(){

}*/

void pc_user_cmd_handle(uint8_t *test_buffer){
	uint8_t cmd = test_buffer[1];
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	connection_task_notification_value_t NotificationValue;
	switch(cmd){
	case PC_CMD_CONNECT_TO_ALL_NODES:
		if(lr_nw_connect_mode == LORA_STATUS_IDLE && lr_nw_disconnect_mode == LORA_STATUS_IDLE){
			// ack to cmd from pc
			pc_user_ack_response();

			lora_network_save_current_work();
			lr_nw_connect_mode = LORA_CONNECT_MODE_ALL;
			NotificationValue.cmd = START_ALL;
			NotificationValue.node_id = 0;
			xQueueSendFromISR(all_connect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
		break;

	case PC_CMD_CONNECT_TO_SPECIFIC_NODE:
		if(lr_nw_connect_mode == LORA_STATUS_IDLE && lr_nw_disconnect_mode == LORA_STATUS_IDLE){
			// ack to cmd from pc
			pc_user_ack_response();

			lora_network_save_current_work();
			lr_nw_connect_mode = LORA_CONNECT_MODE_SPECIFIC;
			NotificationValue.cmd = START_SPECIFIC;
			NotificationValue.node_id = test_buffer[2];
			xQueueSendFromISR(all_connect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
		break;

	case PC_CMD_DISCONNECT_TO_ALL_NODES:
		if(lr_nw_disconnect_mode == LORA_STATUS_IDLE && lr_nw_connect_mode == LORA_STATUS_IDLE){
			// ack to cmd from pc
			pc_user_ack_response();

			lora_network_save_current_work();
			lr_nw_disconnect_mode = LORA_DISCONNECT_MODE_ALL;
			NotificationValue.cmd = STOP_ALL;
			NotificationValue.node_id = 0;
			xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
		break;

	case PC_CMD_DISCONNECT_TO_SPECIFIC_NODE:
		if(lr_nw_disconnect_mode == LORA_STATUS_IDLE && lr_nw_connect_mode == LORA_STATUS_IDLE){
			// ack to cmd from pc
			pc_user_ack_response();

			lora_network_save_current_work();
			lr_nw_disconnect_mode = LORA_DISCONNECT_MODE_SPECIFIC;
			NotificationValue.cmd = STOP_SPECIFIC;
			NotificationValue.node_id = test_buffer[2];
			xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
		break;
	}
}

/**
 * this function is used to handle ack from nodes
 * e.g. ack from node 1 to indicate that node 1 received sucessfully connect cmd from gw
 * **/
void lora_network_cmd_ack_handle(){
	uint8_t k = 0;
	uint8_t responsed = lora_receive_packet_buffer.responsed;
	uint8_t handling_last_buffer = 0;
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	connection_task_notification_value_t NotificationValue;
	lora_packet_t * lora_buffer;

	// check if this is ack for lora_send_buffer or last_lora_send_buffers
	// if this ack is for lora_send_buffer
	HAL_UART_Transmit(&huart2, "Ack handle\n", 11, 2000);
	STM_LOG("Responsed value: %3d\n", responsed, 22);

	if(responsed == lora_send_packet_buffer.packet_id){
		// get node object which the gateway received from
		lora_buffer = &lora_send_packet_buffer;
	}

	else{
		for(k = 0; k < semaphore_count; k++){
			if(responsed == last_lora_send_packet_buffer[k].packet_id){
				lora_buffer = &last_lora_send_packet_buffer[k];
				handling_last_buffer = 1;
				break;
			}
		}
	}

	if(lora_buffer != NULL){
		lora_buffer->responsed = responsed;

		if(lora_buffer->cmd == LORA_CMD_CONNECT || lora_buffer->cmd == LORA_CMD_DISCONNECT){
			lora_node_t *lora_node;
			uint8_t index = 0;
			for(index = 0; index < lr_nw_node_quantity; index++){
				if(NODE_DEVICE[index].id == lora_receive_packet_buffer.source_id){
					lora_node = &NODE_DEVICE[index];
					break;
				}
			}

			if(lora_buffer->cmd == LORA_CMD_CONNECT){
				//xTimerStopFromISR(all_connect_timer, &pxHigherPriorityTaskWoken);
				lora_network_stop_connect_timer();

				// confirm the node lr_nw_node_index is connected and increase lr_nw_connected_node_quantity
				lora_node->connected = 1;

				//int packet_index = bsp_lora_check_cmd_in_node_send_packets(lora_node, LORA_CMD_CONNECT);
				int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_CONNECT, responsed);
				if(packet_index != -1){
					lora_node->last_lora_send_packet[packet_index].responsed = lora_receive_packet_buffer.responsed;
					lora_node->error = 0;
					memcpy((&CONNECTED_NODE[lr_nw_connected_node_quantity]), (lora_node), sizeof(lora_node_t));

					int connect_packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(&CONNECTED_NODE[lr_nw_connected_node_quantity], LORA_CMD_CONNECT, responsed);
					if(connect_packet_index != -1)
						bsp_lora_remove_packet_from_node_send_packets(&CONNECTED_NODE[lr_nw_connected_node_quantity], connect_packet_index);
					/*for(uint8_t i = 0; i < MAX_NODE_PACKET_ITEMS; i++)
						memset(&CONNECTED_NODE[lr_nw_connected_node_quantity].last_lora_send_packet[i], 0, sizeof(lora_packet_t));*/
					lr_nw_connected_node_quantity++;

					// notify to connection task with notification value = ACK
					NotificationValue.cmd = ACK;
					NotificationValue.node_id = lora_node->id;
					NotificationValue.packet_id = responsed;

					HAL_UART_Transmit(&huart2, "Connect notify\n", 15, 2000);
					xQueueSendFromISR(all_connect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
					//portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
				}
			}

			else if(lora_buffer->cmd == LORA_CMD_DISCONNECT){
				//xTimerStopFromISR(all_disconnect_timer, &pxHigherPriorityTaskWoken);
				lora_network_stop_disconnect_timer();

				lora_node->connected = 0;
				int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_DISCONNECT, responsed);
				if(packet_index != -1){
					lora_node->last_lora_send_packet[packet_index].responsed = lora_receive_packet_buffer.responsed;
					lora_node->error = 0;
					for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
						if(CONNECTED_NODE[i].id == lora_node->id){
							for(uint8_t j = i; j < lr_nw_connected_node_quantity - 1; j++){
								memcpy(&CONNECTED_NODE[j], &CONNECTED_NODE[j + 1], sizeof(lora_node_t));
							}
							memset(&CONNECTED_NODE[lr_nw_connected_node_quantity - 1], 0, sizeof(lora_node_t));
							break;
						}
					}

					lr_nw_connected_node_quantity--;
					NotificationValue.cmd = ACK;
					NotificationValue.node_id = lora_node->id;
					NotificationValue.packet_id = responsed;

					HAL_UART_Transmit(&huart2, "Disconnect notify\n", 18, 2000);
					STM_LOG("Node quantity: %3d\n", lr_nw_connected_node_quantity, 19);

					xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
					//portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
				}
			}
		}
		else if(lora_buffer->cmd == LORA_CMD_READ_DATA){
			//xTimerStopFromISR(send_data_timer, &pxHigherPriorityTaskWoken);
			lora_network_stop_read_request_timer();
			for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
				if(CONNECTED_NODE[i].id == lora_receive_packet_buffer.source_id){
					int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(&CONNECTED_NODE[i], LORA_CMD_READ_DATA, responsed);
					if(packet_index != -1){
						CONNECTED_NODE[i].last_lora_send_packet[packet_index].responsed = responsed;
					}

					CONNECTED_NODE[i].error = 0;

					if(lora_receive_packet_buffer.payload_length > 0){
						// receive responsed packet from the node here
						int packet_index1 = bsp_lora_check_cmd_in_node_receive_packets(&CONNECTED_NODE[i], LORA_CMD_ACK);
						int packet_index2 = bsp_lora_get_id_in_node_receive_packets(&CONNECTED_NODE[i], lora_receive_packet_buffer.packet_id);
						int packet_index3 = bsp_lora_get_node_receive_packet_index(&CONNECTED_NODE[i]);

						if((packet_index1 == -1 || packet_index2 == -1) && (packet_index3 != -1)){
							HAL_UART_Transmit(&huart2, "Ack packet has payload length > 0\n", 34, 2000);

							memcpy(&CONNECTED_NODE[i].last_lora_receive_packet[packet_index3], &lora_receive_packet_buffer, sizeof(lora_packet_t));

							NotificationValue.cmd = START_HANDLE_ACK_PACKET;
							NotificationValue.node_id = CONNECTED_NODE[i].id;
							NotificationValue.packet_id = lora_receive_packet_buffer.packet_id;
							xQueueSendFromISR(warning_queue, &NotificationValue, &pxHigherPriorityTaskWoken);

							HAL_UART_Transmit(&huart2, "Notify\n", 7, 2000);
						}
					}


					/*NotificationValue.cmd = ACK;
					NotificationValue.node_id = CONNECTED_NODE[i].id;
					NotificationValue.packet_id = responsed;

					HAL_UART_Transmit(&huart2, "Notify\n", 7, 2000);
					xQueueSendFromISR(send_request_queue, &NotificationValue, &pxHigherPriorityTaskWoken);*/
					//portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
					break;



				}
			}
		}

		/* receive packet from last lora send packet if the ack is for it */
		/*if(handling_last_buffer == 1){
			xSemaphoreTakeFromISR(last_lora_send_packet_counting_semaphore, NULL);
			for(int i = k; i < semaphore_count - 1; i++){
				memcpy(&last_lora_send_packet_buffer[i], &last_lora_send_packet_buffer[i + 1], sizeof(lora_packet_t));
			}
			memset(&last_lora_send_packet_buffer[semaphore_count - 1], 0, sizeof(lora_packet_t));
			handling_last_buffer = 0;
		}*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}


void lora_network_cmd_nack_handle(){
	uint8_t k = 0;
	uint8_t responsed = lora_receive_packet_buffer.responsed;
	uint8_t handling_last_buffer = 0;
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	connection_task_notification_value_t NotificationValue;
	lora_packet_t * lora_buffer;

	HAL_UART_Transmit(&huart2, "Nack handle\n", 12, 2000);
	STM_LOG("Responsed value: %3d\n", responsed, 22);

	// check if this is ack for lora_send_buffer or last_lora_send_buffers
	// if this ack is for lora_send_buffer
	if(responsed == lora_send_packet_buffer.packet_id){
		// get node object which the gateway received from
		lora_buffer = &lora_send_packet_buffer;
	}

	else{
		for(k = 0; k < semaphore_count; k++){
			if(responsed == last_lora_send_packet_buffer[k].packet_id){
				lora_buffer = &last_lora_send_packet_buffer[k];
				handling_last_buffer = 1;
				break;
			}
		}
	}

	if(lora_buffer != NULL){
		lora_buffer->responsed = responsed;

		if(lora_buffer->cmd == LORA_CMD_CONNECT || lora_buffer->cmd == LORA_CMD_DISCONNECT){
			lora_node_t *lora_node;
			uint8_t index = 0;
			for(index = 0; index < lr_nw_node_quantity; index++){
				if(NODE_DEVICE[index].id == lora_receive_packet_buffer.source_id){
					lora_node = &NODE_DEVICE[index];
					break;
				}
			}

			if(lora_buffer->cmd == LORA_CMD_CONNECT){
				xTimerStopFromISR(all_connect_timer, &pxHigherPriorityTaskWoken);

				int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_CONNECT, responsed);
				if(packet_index != -1){
					lora_node->last_lora_send_packet[packet_index].responsed = lora_receive_packet_buffer.responsed;
					lora_node->error = 1;

					// notify to connection task with notification value = NAK
					NotificationValue.cmd = NAK;
					NotificationValue.node_id = 0;
					HAL_UART_Transmit(&huart2, "Nak connect\n", 12, 2000);
					xQueueSendFromISR(all_connect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
				}

			}

			else if(lora_buffer->cmd == LORA_CMD_DISCONNECT){
				xTimerStopFromISR(all_disconnect_timer, &pxHigherPriorityTaskWoken);

				int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(lora_node, LORA_CMD_DISCONNECT, responsed);
				if(packet_index != -1){
					lora_node->last_lora_send_packet[packet_index].responsed = lora_receive_packet_buffer.responsed;
					lora_node->error = 1;
					NotificationValue.cmd = NAK;
					NotificationValue.node_id = 0;
					HAL_UART_Transmit(&huart2, "Nak disconnect\n", 15, 2000);

					xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
				}

			}
		}

		else if(lora_buffer->cmd == LORA_CMD_READ_DATA){
			xTimerStopFromISR(send_data_timer, &pxHigherPriorityTaskWoken);
			for(uint8_t i = 0; i < lr_nw_connected_node_quantity; i++){
				if(CONNECTED_NODE[i].id == lora_receive_packet_buffer.source_id){
					int packet_index = bsp_lora_check_cmd_and_id_in_node_send_packets(&CONNECTED_NODE[i], LORA_CMD_READ_DATA, responsed);
					if(packet_index != -1){
						CONNECTED_NODE[i].last_lora_send_packet[packet_index].responsed = responsed;
						CONNECTED_NODE[i].error = 1;
						NotificationValue.cmd = NAK;
						NotificationValue.node_id = 0;

						HAL_UART_Transmit(&huart2, "Nak send\n", 9, 2000);
						xQueueSendFromISR(send_request_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
						portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
						break;
					}
				}
			}
		}

		if(handling_last_buffer == 1){
			for(uint8_t i = k; i < semaphore_count - 1; i++){
				memcpy(&last_lora_send_packet_buffer[i], &last_lora_send_packet_buffer[i + 1], sizeof(lora_packet_t));
			}
			memset(&last_lora_send_packet_buffer[semaphore_count - 1], 0, sizeof(lora_packet_t));
			handling_last_buffer = 0;
		}
	}
}

void lora_network_cmd_warning_handle(){
	lora_node_t *lora_node = NULL;
	for(uint8_t lora_index = 0; lora_index < lr_nw_connected_node_quantity; lora_index++){
		if(CONNECTED_NODE[lora_index].id == lora_receive_packet_buffer.source_id){
			lora_node = &CONNECTED_NODE[lora_index];
			break;
		}
	}
	if(lora_node != NULL){
		int packet_index = bsp_lora_get_node_receive_packet_index(lora_node);
		if(packet_index != -1){
			STM_LOG("Warning from node %2d\n", lora_node->id, 21);
			memcpy(&lora_node->last_lora_receive_packet[packet_index], &lora_receive_packet_buffer, sizeof(lora_packet_t));

			connection_task_notification_value_t NotificationValue;
			BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
			NotificationValue.cmd = START_RESPONSE_WARNING;
			NotificationValue.node_id = lora_node->id;

			xQueueSendFromISR(warning_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
		}
		// notify to task
	}
}

/**
 * this function is used to handle packet received from lora irq handle function
 * **/
void lora_network_receive_packet_handle(){
	if(lora_receive_packet_buffer.cmd == LORA_CMD_ACK){
		lora_network_cmd_ack_handle();
	}
	else if(lora_receive_packet_buffer.cmd == LORA_CMD_NACK){
		lora_network_cmd_nack_handle();
	}
	else if(lora_receive_packet_buffer.cmd == LORA_CMD_WARNING){
		lora_network_cmd_warning_handle();
	}
}

/**
 * lora irq handle function is used to receive packet through interrupt of DIO0 pin
 * **/
void lora_network_irq_handle(){

	while (HAL_SPI_GetState(myLoRa.hSPIx) != HAL_SPI_STATE_READY);

	uint8_t irqFlags = LoRa_read(&myLoRa, RegIrqFlags);
	if(irqFlags & 0x40) {
		STM_LOG("lora irq flag exact\n", 20, 20);
		bsp_lora_receive_packet();
		// All data have received and saved to lora_receive_packet_buffer
		lora_network_receive_packet_handle();
	}
	else{
		STM_LOG("lora irq flag wrong\n", 20, 20);
		//lora_network_send_request_start_scheduler();
	}
}


void lora_network_timer_handle(){
	connection_task_notification_value_t NotificationValue;
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	if(lr_nw_connect_timer_enable == 1){
		lr_nw_connect_timer_count++;

		if(lr_nw_connect_timer_count >= 10){
			lora_network_stop_connect_timer();

			NotificationValue.cmd = TIMEOUT;
			NotificationValue.node_id = 0;
			HAL_UART_Transmit(&huart2, "Connect timeout\n", 15, 2000);

			xQueueSendFromISR(all_connect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
		}
	}

	if(lr_nw_disconnect_timer_enable == 1){
		lr_nw_disconnect_timer_count++;

		if(lr_nw_disconnect_timer_count >= 10){
			lora_network_stop_disconnect_timer();

			NotificationValue.cmd = TIMEOUT;
			NotificationValue.node_id = 0;
			HAL_UART_Transmit(&huart2, "Disconnect timeout\n", 18, 2000);

			xQueueSendFromISR(all_disconnect_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
		}
	}

	if(lr_nw_read_request_timer_enable == 1){
		lr_nw_read_request_timer_count++;

		if(lr_nw_read_request_timer_count >= 10){
			lora_network_stop_read_request_timer();

			NotificationValue.cmd = TIMEOUT;
			NotificationValue.node_id = 0;
			HAL_UART_Transmit(&huart2, "Send request timeout\n", 20, 2000);

			xQueueSendFromISR(send_request_queue, &NotificationValue, &pxHigherPriorityTaskWoken);
		}
	}
}
