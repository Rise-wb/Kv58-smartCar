#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "include.h"

//引脚定义
#define NRF2401_CE_PIN    &PTC8
#define NRF2401_CSN_PIN   &PTC9
#define NRF2401_IRQ_PIN   &PTC5
#define NRF2401_SCK_PIN   &PTC6
#define NRF2401_SDO_PIN   &PTC7
#define NRF2401_SDI_PIN   &PTC4
//位宏
  #define NRF2401_CE_H    gpio_set(NRF2401_CE_PIN,1) 
  #define NRF2401_CE_L    gpio_set(NRF2401_CE_PIN,0) 

	#define NRF2401_CSN_H   gpio_set(NRF2401_CSN_PIN,1)
	#define NRF2401_CSN_L   gpio_set(NRF2401_CSN_PIN,0)

	#define NRF2401_IRQ     gpio_get(NRF2401_IRQ_PIN)
	
	#define NRF2401_SCK_H   gpio_set(NRF2401_SCK_PIN,1)
	#define NRF2401_SCK_L   gpio_set(NRF2401_SCK_PIN,0)

	#define NRF2401_SDO_H   gpio_set(NRF2401_SDO_PIN,1)
	#define NRF2401_SDO_L   gpio_set(NRF2401_SDO_PIN,0)

	#define NRF2401_SDI     gpio_get(NRF2401_SDI_PIN)
//返回常量
#define NRF_OK      (0x00)
#define NRF_ERR     (0x01)
#define MAX_TX  		(0x10)  //达到最大发送次数中断
#define TX_OK   		(0x20)  //TX发送完成中断
#define RX_OK   		(0x40)  //接收到数据中断

//********************************************************************************************************************// 
// SPI(nRF24L01) 指令
#define READ_REG    0x00   // 读配置寄存器
#define WRITE_REG   0x20   // 写配置寄存器
#define RD_RX_PLOAD 0x61   // 读取RX FIFO中的数据
#define WR_TX_PLOAD 0xA0   // 向TX FIFO中写入数据
#define FLUSH_TX    0xE1   // 清除TX FIFO中的数据 应用于发射模式下
#define FLUSH_RX    0xE2   // 清除RX FIFO中的数据 应用于接收模式下
#define REUSE_TX_PL 0xE3   // 重新使用上一包有效数据
#define NOP         0xFF   // 保留
//********************************************************************************************************************// 
// SPI(nRF24L01) 寄存器(地址)
#define CONFIG      0x00  //配置发送状态，CRC校验模式以及发收发状态响应方式
#define EN_AA       0x01  //自动应答功能设置
#define EN_RXADDR   0x02  //可用信道设置
#define SETUP_AW    0x03  //收发地址宽度设置
#define SETUP_RETR  0x04  //自动重发功能设设置
#define RF_CH       0x05  //工作频率设定
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17

#define STATUS_MAX_RT 0x10
#define STATUS_TX_DS  0x20
#define STATUS_RX_DR  0x40


//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  3  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  3  	//32字节的用户数据宽度
//本构件所实现的接口函数
extern uint8_t NRF2401_Init(void);
extern void NRF2401_SetRXMode(void);
extern void NRF2401_SetTXMode(void);
extern uint8_t NRF2401_SendData(uint8_t *txbuf);
extern uint8_t NRF2401_RecData(uint8_t *rxbuf);
#endif
