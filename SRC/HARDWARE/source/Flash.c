#include "Flash.h"
#include "fsl_common.h"
 

#define     FCMD        FTFE->FCCOB0        //FTFL ����

#define     FADDR2      FTFE->FCCOB1         //Flash address [23:16]
#define     FADDR1      FTFE->FCCOB2         //Flash address [15:8]
#define     FADDR0      FTFE->FCCOB3         //Flash address [7:0]

#define     FDATA0      FTFE->FCCOB4         //Data Byte 0       //ע��һ�㣬4�ֽ�����FDATA3�����λ��FDATA0�����λ
#define     FDATA1      FTFE->FCCOB5         //Data Byte 1
#define     FDATA2      FTFE->FCCOB6         //Data Byte 2
#define     FDATA3      FTFE->FCCOB7         //Data Byte 3
#define     FDATA4      FTFE->FCCOB8         //Data Byte 4
#define     FDATA5      FTFE->FCCOB9         //Data Byte 5
#define     FDATA6      FTFE->FCCOBA         //Data Byte 6
#define     FDATA7      FTFE->FCCOBB         //Data Byte 7

//FCMD ����
#define     RD1BLK    0x00   // ������Flash
#define     RD1SEC    0x01   // ����������
#define     PGMCHK    0x02   // д����
#define     RDRSRC    0x03   // ��Ŀ������(4�ֽ�)
#define     PGM4      0x06   // д�볤��(4�ֽ�)
#define     ERSBLK    0x08   // ��������Flash
#define     ERSSCR    0x09   // ����Flash����
#define     PGMSEC    0x0B   // д������
#define     RD1ALL    0x40   // �����еĿ�
#define     RDONCE    0x41   // ֻ��һ��
#define     PGMONCE   0x43   // ֻдһ��
#define     ERSALL    0x44   // �������п�
#define     VFYKEY    0x45   // ��֤���ŷ���Կ��
#define     PGMPART   0x80   // д�����
#define     SETRAM    0x81   // �趨FlexRAM����

int f_k[4]={1,1,1,1};

/*!
 *  @brief      Flash����
 *  @return     ����ִ�н��(1�ɹ���0ʧ��)
 *  @since      v5.0
 */
__ramfunc uint8_t flash_cmd()
{
    //д FTFL_FSTAT ���� Flash����

    FTFE->FSTAT =    (0
                     |  FTFE_FSTAT_CCIF_MASK        // ָ����ɱ�־(д1��0)
                     |  FTFE_FSTAT_RDCOLERR_MASK    // ����ͻ�����־(д1��0)
                     |  FTFE_FSTAT_ACCERR_MASK      // ���ʴ����־λ(д1��0)
                     |  FTFE_FSTAT_FPVIOL_MASK      // �Ƿ����ʱ�����־λ(д1��0)
                    );

    while(!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));    // �ȴ��������

    // �������־
    if( FTFE->FSTAT & (FTFE_FSTAT_ACCERR_MASK | FTFE_FSTAT_RDCOLERR_MASK | FTFE_FSTAT_FPVIOL_MASK | FTFE_FSTAT_MGSTAT0_MASK))
    {
        return 0 ;                                  //ִ���������
    }
    else
    {
        return 1;                                   //ִ������ɹ�
    }

}
/*!
 *  @brief      ��ʼ��flash
 *  @since      v5.0
 */
__ramfunc void flash_init(void)
{
   // ���FlashԤ��ȡ������
    FMC->PFB0CR |= FMC_PFB0CR_S_INV_MASK;      //
    //FMC_PFB23CR |= FMC_PFB23CR_S_B_INV_MASK;

    while(!(FTFE->FSTAT & FTFE_FSTAT_CCIF_MASK));    // �ȴ��������

    FTFE->FSTAT =    (0
                     |  FTFE_FSTAT_CCIF_MASK        // ָ����ɱ�־(д1��0)
                     |  FTFE_FSTAT_RDCOLERR_MASK    // ����ͻ�����־(д1��0)
                     |  FTFE_FSTAT_ACCERR_MASK      // ���ʴ����־λ(д1��0)
                     |  FTFE_FSTAT_FPVIOL_MASK      // �Ƿ����ʱ�����־λ(д1��0)
                    );

    lptmr_delay_ms(10);
}
/*!
 *  @brief      ����ָ��flash����
 *  @param      sector_num    �����ţ�K60N512ʵ��ʹ��0~255��
 *  @return     ִ�н��(1�ɹ���0ʧ��)
 *  @since      v5.0
 *  Sample usage:       flash_erase_sector(127);        //��������127
 */
__ramfunc uint8_t flash_erase_sector(uint16_t sector_num)
{
    uint32_t addr = sector_num * FLASH_SECTOR_SIZE;

    // ���ò�������
    FCMD = ERSSCR;
    
    // ����Ŀ���ַ
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
 *  @brief      д�볤�ֽ����ݵ� flashָ����ַ
 *  @param      sector_num      �����ţ�0 ~ FLASH_SECTOR_NUM��
 *  @param      offset          д�������ڲ�ƫ�Ƶ�ַ��0~2043 �� 4�ı�����
 *  @param      data            ��Ҫд�������
 *  @return     ִ�н��(1�ɹ���0ʧ��)
 *  @since      v5.0
 *  Sample usage:       flash_write(127,0,0xFFFFFFFE);        //����127,ƫ�Ƶ�ַΪ0,д������:0xFFFFFFFE
 */
__ramfunc uint8_t flash_write(uint16_t sector_num, uint16_t offset, FLASH_WRITE_TYPE data)
{
    uint32_t addr = sector_num * FLASH_SECTOR_SIZE  + offset ;
    uint32_t tmpdata;
    
    // ����Ŀ���ַ
    FADDR2 = ((Dtype *)&addr)->B[2];
    FADDR1 = ((Dtype *)&addr)->B[1];
    FADDR0 = ((Dtype *)&addr)->B[0];

    // ���� ��32λ����
    tmpdata = (uint32_t)data;

    FDATA0 = ((Dtype *)&tmpdata)->B[3];                    // ����д������
    FDATA1 = ((Dtype *)&tmpdata)->B[2];
    FDATA2 = ((Dtype *)&tmpdata)->B[1];
    FDATA3 = ((Dtype *)&tmpdata)->B[0];


    // ���ò�������
    FCMD = PGM4;


    if(flash_cmd() == 0)
    {
        return 0;
    }

    return 1;  //�ɹ�ִ��

}

/*!
 *  @brief      д�����ݻ������� flashָ����ַ
 *  @param      sector_num      �����ţ�K60N512ʵ��ʹ��0~255��
 *  @param      offset          д�������ڲ�ƫ�Ƶ�ַ��0~2043 �� 4�ı�����
 *  @param      buf             ��Ҫд������ݻ������׵�ַ
 *  @return     ִ�н��(1�ɹ���0ʧ��)
 *  @since      v5.0
 *  Sample usage:           uint32 buff[10];
                            flash_write_buf(127,0,sizeof(buff),buff);        //����127,ƫ�Ƶ�ַΪ0,д������Դ��ַ:buff,��Ŀsizeof(buff)
 */

__ramfunc uint8_t flash_write_buf(uint16_t sector_num, uint16_t offset, uint16_t cnt, uint8_t *buf)
{
    uint32_t  size;

    uint32_t  addr;
    uint32_t  data;

    addr = sector_num * FLASH_SECTOR_SIZE  + offset;      //�����ַ

    // ����д������

    FCMD = PGM4;

    //FCMD = PGM8;


    for(size = 0; size < cnt; size += FLASH_ALIGN_ADDR )
    {

        // ����Ŀ���ַ
        FADDR2 = ((Dtype *)&addr)->B[2];
        FADDR1 = ((Dtype *)&addr)->B[1];
        FADDR0 = ((Dtype *)&addr)->B[0];

        // ���� ��32λ����
        data =  *(uint32_t *)buf;

        FDATA0 = ((Dtype *)&data)->B[3];                    // ����д������
        FDATA1 = ((Dtype *)&data)->B[2];
        FDATA2 = ((Dtype *)&data)->B[1];
        FDATA3 = ((Dtype *)&data)->B[0];

#if defined(MK60F15)
        // ���� ��32λ����
        data = *(uint32 *)(buf+4);

        FDATA4 = ((Dtype *)&data)->B[3];                    // ����д������
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
    return 1;  //�ɹ�ִ��
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

