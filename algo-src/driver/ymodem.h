#ifndef __YMODEM_H
#define __YMODEM_H

#include "stm32f10x_conf.h"
#include "string.h"
#include "bsp_usart.h"	 
#include "rs485.h"	 

#define uart_log  printf

#define YMODEM_SOH		0x01  // start of 128
#define YMODEM_STX		0x02  // start of 1024
#define YMODEM_EOT		0x04  // end of transmission
#define YMODEM_ACK		0x06  // positive acknowledge
#define YMODEM_NAK		0x15  // negative acknowledge
#define YMODEM_CA		0x18  // cancle终止
#define YMODEM_C		0x43  // control character 'C'
#define YMODEM_END      0x4F  // control character 'O'关闭传输

#define FLASH_SECTOR_SIZE       1024
#define FLASH_SECTOR_NUM        512    // 512k
#define FLASH_START_ADDR        ((uint32_t)0x08000000)
#define FLASH_END_ADDR          ((uint32_t)(0x08000000 + FLASH_SECTOR_NUM * FLASH_SECTOR_SIZE))

#define DOWN_SECTOR_ADDR         0x08042000
#define ERASE_SECTORS	        ((FLASH_END_ADDR - DOWN_SECTOR_ADDR) / FLASH_SECTOR_SIZE)   // 128k - 16 - 56 = 56k

#define MAX_QUEUE_SIZE            1200

#define SETTING_BOOT_STATE      0x08003000
#define UPDATE_PROGRAM_STATE    2
#define UPDATE_SUCCESS_STATE    3

typedef enum {
	NONE,
	BUSY,
	START_PROGRAM,
	UPDATE_PROGRAM,
	UPDATE_SUCCESS
} process_status;

typedef void (*ymodem_callback)(process_status);

typedef struct {
	process_status process;
	uint8_t status;
	uint8_t id;
	uint32_t addr;
	uint32_t filesize;
	char filename[32];
	ymodem_callback cb;
} ymodem_t;

//顺序循环队列的结构体定义如下：
typedef struct
{
	uint8_t queue[MAX_QUEUE_SIZE];
	int rear;  //队尾指针
	int front;  //队头指针
	int count;  //计数器
} seq_queue_t; 

typedef  void (*jump_callback)(void);

typedef struct 
{
	uint8_t data[1200];
	uint16_t len;
} download_buf_t;

extern seq_queue_t rx_queue;
extern download_buf_t down_buf;

void queue_initiate(seq_queue_t *Q);
int queue_not_empty(seq_queue_t *Q);
int queue_delete(seq_queue_t *Q, uint8_t *d);

uint8_t mcu_flash_erase(uint32_t addr, uint8_t count);
uint8_t mcu_flash_write(uint32_t addr, uint8_t *buffer, uint32_t length);
void mcu_flash_read(uint32_t addr, uint8_t *buffer, uint32_t length);

void set_ymodem_status(process_status process);
process_status get_ymodem_status(void);
void ymodem_start(ymodem_callback cb);
void ymodem_recv(download_buf_t *p);
void ymodem_init(void);
void ymodem_handle(void);
void timer_init(void);

#endif
