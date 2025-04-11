#ifndef LORA_NETWORK_H
#define LORA_NETWORK_H

#include "bsp_lora.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"


/* network define section */
#define MAX_NODE_QUANTITY 7


/* device's class define section */
#define DEVICE_CLASS_A 0
#define DEVICE_CLASS_B 1
#define DEVICE_CLASS_C 2

#define DEVICE_CLASS_A_TIME_PERIOD 4000
#define DEVICE_CLASS_B_TIME_PERIOD 3000
#define DEVICE_CLASS_C_TIME_PERIOD 2000

/* number of software timers to used */
#define NUMBER_OF_SWTIMER 10

/* number of last send buffers to used */
#define NUMBER_OF_LAST_LORA_SEND_PACKET 3

#define SEND_REQUEST_TIME_PERIOD 15000

#define TTL_OF_PACKET 3

/* command received from pc */
enum pc_cmd{
	PC_CMD_CONNECT_TO_ALL_NODES = 0,
	PC_CMD_CONNECT_TO_SPECIFIC_NODE,
	PC_CMD_DISCONNECT_TO_ALL_NODES,
	PC_CMD_DISCONNECT_TO_SPECIFIC_NODE,
	PC_CMD_INITIALISE_NODE_QUANTITY,
};

/* the data frame between stm32 and esp32:
	| length | cmd	  | node's id | connection status(1 byte), transfer status(1 byte), machine status(1 byte), 5 byte reserved, 32 byte data |
	| 1 byte | 1 byte |  1 byte   |                                     40 byte                                                               |

special case:
 */
 typedef struct uart_data_frame_s{
	uint8_t len;
	uint8_t cmd;
	uint8_t node_id;
	uint8_t raw_data[40];
 }uart_data_frame_t;

enum uart_cmd{
	UART_CMD_INITIALISE_NODE_QUANTITY = 0,
	UART_CMD_HANDLE_DATA_READ_FROM_NODE,
	UART_CMD_HANDLE_WARNING_NODE,
};

enum lora_network_work_status{
	LORA_STATUS_IDLE = 0,
	LORA_CONNECT_MODE_ALL,
	LORA_CONNECT_MODE_SPECIFIC,
	LORA_DISCONNECT_MODE_ALL,
	LORA_DISCONNECT_MODE_SPECIFIC,
};

typedef enum connection_task_notification_cmd_e{
	START_ALL = 0,
	START_SPECIFIC,
	STOP_ALL,
	STOP_SPECIFIC,
	START_SEND_REQUEST,
	START_RESPONSE_WARNING,
	START_HANDLE_ACK_PACKET,
	ACK,
	NAK,
	TIMEOUT,
}connection_task_notification_cmd_t;

enum software_timer_id{
	SW_TIMER_CONNECT_ALL = 0,
	SW_TIMER_DISCONNECT_ALL,
	SW_TIMER_SEND_REQUEST,
	SW_TIMER_SEND_DATA,
};

typedef struct connection_task_notification_value_s{
	connection_task_notification_cmd_t cmd;
	uint8_t value;
}connection_task_notification_value_t;

void lora_network_init(uint8_t node_quantity);

void lora_network_increase_tickcount();

void lora_network_send_request_scheduler();

void pc_user_cmd_handle(uint8_t *test_buffer);

#endif
