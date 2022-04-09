#ifndef _Flash_H
#define _Flash_H

#include "include.h"
#define     __ramfunc
#define     FLASH_SECTOR_SIZE       (8*1024)                //扇区大小 为 2k 字节
#define     FLASH_SECTOR_NUM        (128)                   //扇区数

#define     FLASH_ALIGN_ADDR        4                       //地址对齐整数倍
typedef     uint32_t                  FLASH_WRITE_TYPE;       //flash_write 函数写入 的数据类型
#define  		Address_adc  FLASH_SECTOR_NUM-1      //起始地址

/*
 * 定义带位域的联合体类型
 */
#pragma anon_unions
typedef union
{
    uint32_t  DW;
    uint16_t  W[2];
    uint8_t   B[4];
    struct
    {
        uint32_t b0: 1;
        uint32_t b1: 1;
        uint32_t b2: 1;
        uint32_t b3: 1;
        uint32_t b4: 1;
        uint32_t b5: 1;
        uint32_t b6: 1;
        uint32_t b7: 1;
        uint32_t b8: 1;
        uint32_t b9: 1;
        uint32_t b10: 1;
        uint32_t b11: 1;
        uint32_t b12: 1;
        uint32_t b13: 1;
        uint32_t b14: 1;
        uint32_t b15: 1;
        uint32_t b16: 1;
        uint32_t b17: 1;
        uint32_t b18: 1;
        uint32_t b19: 1;
        uint32_t b20: 1;
        uint32_t b21: 1;
        uint32_t b22: 1;
        uint32_t b23: 1;
        uint32_t b24: 1;
        uint32_t b25: 1;
        uint32_t b26: 1;
        uint32_t b27: 1;
        uint32_t b28: 1;
        uint32_t b29: 1;
        uint32_t b30: 1;
        uint32_t b31: 1;
    };
} Dtype;    //sizeof(Dtype) 为 4

extern int f_k[4];

//内部接口
extern __ramfunc uint8_t flash_cmd(void);
extern __ramfunc void flash_init(void);
extern __ramfunc uint8_t flash_erase_sector(uint16_t sector_num);
extern __ramfunc uint8_t flash_write(uint16_t sector_num, uint16_t offset, FLASH_WRITE_TYPE data);
extern __ramfunc uint8_t flash_write_buf(uint16_t sector_num, uint16_t offset, uint16_t cnt, uint8_t *buf);
#define     flash_read(sectorNo,offset,type)        (*(type *)((uint32_t)(((sectorNo)*FLASH_SECTOR_SIZE) + (offset))))

extern void ADC_Write(void);

extern void ADC_Read(void);

#endif
