#include "Flash.h"
#include "fsl_common.h"
 

#define     FCMD        FTFE->FCCOB0        //FTFL 命令

#define     FADDR2      FTFE->FCCOB1         //Flash address [23:16]
#define     FADDR1      FTFE->FCCOB2         //Flash address [15:8]
#define     FADDR0      FTFE->FCCOB3         //Flash address [7:0]

#define     FDATA0      FTFE->FCCOB4         //Data Byte 0       //注意一点，4字节排序，FDATA3是最低位，FDATA0是最高位
#define     FDATA1      FTFE->FCCOB5         //Data Byte 1
#define     FDATA2      FTFE->FCCOB6         //Data Byte 2
#define     FDATA3      FTFE->FCCOB7         //Data Byte 3
#define     FDATA4      FTFE->FCCOB8         //Data Byte 4
#define     FDATA5      FTFE->FCCOB9         //Data Byte 5
#define     FDATA6      FTFE->FCCOBA         //Data Byte 6
#define     FDATA7      FTFE->FCCOBB         //Data Byte 7

//FCMD 命令
#define     RD1BLK    0x00   // 读整块Flash
#define     RD1SEC    0x01   // 读整个扇区
#define     PGMCHK    0x02   // 写入检查
#define     RDRSRC    0x03   // 读目标数据(4字节)
#define     PGM4      0x06   // 写入长字(4字节)
#define     ERSBLK    0x08   // 擦除整块Flash
#define     ERSSCR    0x09   // 擦除Flash扇区
#define     PGMSEC    0x0B   // 写入扇区
#define     RD1ALL    0x40   // 读所有的块
#define     RDONCE    0x41   // 只读一次
#define     PGMONCE   0x43   // 只写一次
#define     ERSALL    0x44   // 擦除所有块
#define     VFYKEY    0x45   // 验证后门访问钥匙
#define     PGMPART   0x80   // 写入分区
#define     SETRAM    0x81   // 设定FlexRAM功能

int f_k[4]={1,1,1,1};

/*!
 *  @brief      Flash命令
 *  @return     命令执行结果(1成功，0失败)
 *  @since      v5.0
 */
__ramfunc uint8_t flash_cmd()
{
    //写 FTFL_FSTAT 启动 Flash命令

    FTFE->FSTAT =    (0
                     |  FTFE_FSTAT_CCIF_MASK        // 指令完成标志(写1清0)
                     |  FTFE_FSTAT_RDCOLERR_MASK    // 读冲突错误标志(写1清0)
                     |  FTFE_FSTAT_ACCERR_MASK      // 访问错误标志位(写1清0)
                     |  FTFE_FSTAT_FPVIOL_MASK      // 非法访问保护标志位(写1清0)
                    );

    while(!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));    // 等待命令完成

    // 检查错误标志
    if( FTFE->FSTAT & (FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_RDCOLERR_MASK | FTFE_FSTAT_FPVIOL_MASK | FTFE_FSTAT_MGSTAT0_MASK))
    {
        return 0 ;                                  //执行命令出错
    }
    else
    {
        return 1;                                   //执行命令成功
    }

}
/*!
 *  @brief      初始化flash
 *  @since      v5.0
 */
__ramfunc void flash_init(void)
{
   // 清除Flash预读取缓冲区
    FMC->PFB0CR |= FMC_PFB0CR_S_INV_MASK;      //
    //FMC_PFB23CR |= FMC_PFB23CR_S_B_INV_MASK;

    while(!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));    // 等待命令完成

    FTFE->FSTAT =    (0
                     |  FTFE_FSTAT_CCIF_MASK        // 指令完成标志(写1清0)
                     |  FTFE_FSTAT_RDCOLERR_MASK    // 读冲突错误标志(写1清0)
                     |  FTFE_FSTAT_ACCERR_MASK      // 访问错误标志位(写1清0)
                     |  FTFE_FSTAT_FPVIOL_MASK      // 非法访问保护标志位(写1清0)
                    );

    lptmr_delay_ms(10);
}
/*!
 *  @brief      擦除指定flash扇区
 *  @param      sector_num    扇区号（K60N512实际使用0~255）
 *  @return     执行结果(1成功，0失败)
 *  @since      v5.0
 *  Sample usage:       flash_erase_sector(127);        //擦除扇区127
 */
__ramfunc uint8_t flash_erase_sector(uint16_t sector_num)
{
    uint32_t addr = sector_num * FLASH_SECTOR_SIZE;

    // 设置擦除命令
    FCMD = ERSSCR;
    
    // 设置目标地址
    FADDR2 = ((Dtype *)&addr)->B[2];
    FADDR1 = ((Dtype *)&addr)->B[1];
    FADDR0 = ((Dtype *)&addr)->B[0];

    if(flash_cmd() == 0)
    {
        return 0;
    }

    if(sector_num == 0)
    {
        return flash_write(sector_num,0x00040C,0xFFFFFFFE);
				//return flash_write(sector_num,0x000408,0xFFFFFFFEFFFFFFFF );
    }

    return 1;
}

/*!
 *  @brief      写入长字节数据到 flash指定地址
 *  @param      sector_num      扇区号（0 ~ FLASH_SECTOR_NUM）
 *  @param      offset          写入扇区内部偏移地址（0~2043 中 4的倍数）
 *  @param      data            需要写入的数据
 *  @return     执行结果(1成功，0失败)
 *  @since      v5.0
 *  Sample usage:       flash_write(127,0,0xFFFFFFFE);        //扇区127,偏移地址为0,写入数据:0xFFFFFFFE
 */
__ramfunc uint8_t flash_write(uint16_t sector_num, uint16_t offset, FLASH_WRITE_TYPE data)
{
    uint32_t addr = sector_num * FLASH_SECTOR_SIZE  + offset ;
    uint32_t tmpdata;
    
    // 设置目标地址
    FADDR2 = ((Dtype *)&addr)->B[2];
    FADDR1 = ((Dtype *)&addr)->B[1];
    FADDR0 = ((Dtype *)&addr)->B[0];

    // 设置 低32位数据
    tmpdata = (uint32_t)data;

    FDATA0 = ((Dtype *)&tmpdata)->B[3];                    // 设置写入数据
    FDATA1 = ((Dtype *)&tmpdata)->B[2];
    FDATA2 = ((Dtype *)&tmpdata)->B[1];
    FDATA3 = ((Dtype *)&tmpdata)->B[0];


    // 设置擦除命令
    FCMD = PGM4;


    if(flash_cmd() == 0)
    {
        return 0;
    }

    return 1;  //成功执行

}

/*!
 *  @brief      写入数据缓冲区到 flash指定地址
 *  @param      sector_num      扇区号（K60N512实际使用0~255）
 *  @param      offset          写入扇区内部偏移地址（0~2043 中 4的倍数）
 *  @param      buf             需要写入的数据缓冲区首地址
 *  @return     执行结果(1成功，0失败)
 *  @since      v5.0
 *  Sample usage:           uint32 buff[10];
                            flash_write_buf(127,0,sizeof(buff),buff);        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
 */

__ramfunc uint8_t flash_write_buf(uint16_t sector_num, uint16_t offset, uint16_t cnt, uint8_t *buf)
{
    uint32_t  size;

    uint32_t  addr;
    uint32_t  data;

    addr = sector_num * FLASH_SECTOR_SIZE  + offset;      //计算地址

    // 设置写入命令

    FCMD = PGM4;

    //FCMD = PGM8;


    for(size = 0; size < cnt; size += FLASH_ALIGN_ADDR )
    {

        // 设置目标地址
        FADDR2 = ((Dtype *)&addr)->B[2];
        FADDR1 = ((Dtype *)&addr)->B[1];
        FADDR0 = ((Dtype *)&addr)->B[0];

        // 设置 低32位数据
        data =  *(uint32_t *)buf;

        FDATA0 = ((Dtype *)&data)->B[3];                    // 设置写入数据
        FDATA1 = ((Dtype *)&data)->B[2];
        FDATA2 = ((Dtype *)&data)->B[1];
        FDATA3 = ((Dtype *)&data)->B[0];

#if defined(MK60F15)
        // 设置 高32位数据
        data = *(uint32 *)(buf+4);

        FDATA4 = ((Dtype *)&data)->B[3];                    // 设置写入数据
        FDATA5 = ((Dtype *)&data)->B[2];
        FDATA6 = ((Dtype *)&data)->B[1];
        FDATA7 = ((Dtype *)&data)->B[0];
#endif

        if(flash_cmd() == 0)
        {
            return 0;
        }

        addr += FLASH_ALIGN_ADDR;
        buf += FLASH_ALIGN_ADDR;
    }
    return 1;  //成功执行
}

void ADC_Write()
{
   flash_erase_sector(Address_adc); 
   flash_write(Address_adc,0,f_k[0]);
   flash_write(Address_adc,4,f_k[1]);
   flash_write(Address_adc,8,f_k[2]);
   flash_write(Address_adc,12,f_k[3]);
}

void ADC_Read()
{
   f_k[0]=flash_read(Address_adc,0,int);
   f_k[1]=flash_read(Address_adc,4,int);
   f_k[2]=flash_read(Address_adc,8,int);
   f_k[3]=flash_read(Address_adc,12,int);
}

