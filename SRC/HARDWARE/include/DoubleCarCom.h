#ifndef DCC_H
#define DCC_H
#include "include.h"

#define POSFRONT 0    //ǰ��
#define POSHIND  1    //��
//#define CGSTAY   0    //����
//#define CGCHAN   1    //��ʼ����
//#define FINISH5  1    //5�����
//#define GOON5    0    //5�μ���
//#define STOP     1    //ͣ��
//#define UNSTOP   0    //����
//#define CHANEN   1    //������
//#define CHANDIS  0    //��ֹ����
//#define RUN      1    //����
//#define RUNOVER  0    //��ʱͣ��
//#define DIRLEFT  0    //��ת
//#define DIRRIGHT 1    //��ת
#define DTOP     0x4C    //ͷ������
#define DBTM     0xC4    //βУ����
#define CLEAN    0xFF    //���
#define RUN      0x00    //����
#define RUNOVER  0x01    //��ʱ����ǰ�����ƽ���
#define STOP     0x02    //����յ�ͣ��
#define DIRLEFT  0x03    //�󳬳�
#define DIRRIGHT 0x04    //�ҳ���
#define CGREADY  0x05    //�󳵼�⵽ǰ��
#define FINISH5  0x06    //��ɳ�������
#define CIRLEFT  0x07    //�����㷽����
#define CIRRIGHT 0x08    //�����㷽����
#define CIRBREAK 0x09    //���������ܾ�
    
typedef struct 
{
    uint8_t isFrontPos:1;   //���λ�ñ�� 1ǰ�� 0��
    uint8_t isChangePos:1;  //��ʼ������� 1���� 0����
    uint8_t isFinish:1;     //5�γ�����ɱ�� 1��� 0����
    uint8_t isStop:1;       //��⵽������ͣ����־ 1��⵽ 0����
    uint8_t isChangeEn:1;   //������ٱ��  1���� 0��ֹ
    uint8_t isGo:1;         //�������      1���� 0ͣ��
    uint8_t runDir:1;       //����
    uint8_t reserved:1;     //����
}CarState;

extern CarState thisCar;  
extern CarState* thiC ;
extern CarState anotCar ;
extern CarState* anoC;

extern uint8_t thisCarState;
extern uint8_t anotCarState;

extern void CommunicationInit(void);//��ʼ��  
extern void TransmitData(void);    //����
extern void ReceviceData(void);    //����
#endif  //DCC_H
