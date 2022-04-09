#ifndef DCC_H
#define DCC_H
#include "include.h"

#define POSFRONT 0    //前车
#define POSHIND  1    //后车
//#define CGSTAY   0    //保持
//#define CGCHAN   1    //开始超车
//#define FINISH5  1    //5次完成
//#define GOON5    0    //5次继续
//#define STOP     1    //停车
//#define UNSTOP   0    //继续
//#define CHANEN   1    //允许超车
//#define CHANDIS  0    //禁止超车
//#define RUN      1    //发车
//#define RUNOVER  0    //计时停车
//#define DIRLEFT  0    //左转
//#define DIRRIGHT 1    //右转
#define DTOP     0x4C    //头检验码
#define DBTM     0xC4    //尾校验码
#define CLEAN    0xFF    //清除
#define RUN      0x00    //起跑
#define RUNOVER  0x01    //计时或者前车控制结束
#define STOP     0x02    //检测终点停车
#define DIRLEFT  0x03    //左超车
#define DIRRIGHT 0x04    //右超车
#define CGREADY  0x05    //后车检测到前车
#define FINISH5  0x06    //完成超车次数
#define CIRLEFT  0x07    //环道点方向左
#define CIRRIGHT 0x08    //环道点方向右
#define CIRBREAK 0x09    //环道超车拒绝
    
typedef struct 
{
    uint8_t isFrontPos:1;   //相对位置标记 1前车 0后车
    uint8_t isChangePos:1;  //开始超车标记 1超车 0保持
    uint8_t isFinish:1;     //5次超车完成标记 1完成 0继续
    uint8_t isStop:1;       //检测到起跑线停车标志 1检测到 0继续
    uint8_t isChangeEn:1;   //允许加速标记  1允许 0禁止
    uint8_t isGo:1;         //发车标记      1发车 0停车
    uint8_t runDir:1;       //方向
    uint8_t reserved:1;     //保留
}CarState;

extern CarState thisCar;  
extern CarState* thiC ;
extern CarState anotCar ;
extern CarState* anoC;

extern uint8_t thisCarState;
extern uint8_t anotCarState;

extern void CommunicationInit(void);//初始化  
extern void TransmitData(void);    //发送
extern void ReceviceData(void);    //接收
#endif  //DCC_H
