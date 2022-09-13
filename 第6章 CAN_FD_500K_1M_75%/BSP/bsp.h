#ifndef __BSP_H
#define __BSP_H	 
#include "gd32C10x.h"
#include "string.h"
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

extern __IO uint32_t  iTimeCmt;
extern __IO uint32_t  iTime5Cmt;   //定时器5计数器
extern __IO uint32_t   iTime6Cmt;   //定时器6计数器
extern __IO uint32_t 			iTim1Value;//定时器1 输入捕获溢出值


extern  u8 BUF_MO[1024*4];
extern  u8 BUF_MOCAN[1024*4];
extern  u8 ReDataStr[1024*7];//接收缓冲区


extern u8 j1850_Hbit[500];//J1850   高电平时间
extern u8 j1850_Lbit[500];//J1850   低电平时间

extern __IO u16 iHBitSum;//高电平计数
extern __IO u16 iLBitSum;//低电平 计数
extern __IO u8 iPartValue;

extern  __IO u32 iReCount;//缓冲区帧数		
extern  __IO u8  iKDataMode;
extern  __IO u8  iTimeFlag;
extern __IO uint8_t  iKwpIndexTime;//KWP 帧间隔时间
extern __IO u8 iJ1850EofFlag;//1850 帧结束标记   1=帧结束  

extern __IO u8 iEcuFlag;//是否是模拟ECU  00采集 数据 80 ECU 模拟
extern __IO u16 iSendComValue;
extern __IO u16 iCanSendSum ;

extern __IO u8 Fd_Working_mode; //工作模式


extern __IO u8 iCan30flag;//接收到30帧 标记 00=需要等待30帧，80=已经接收到30帧 
extern __IO u8 iCan30Sum;//30帧后允许发送的多帧 帧数
extern __IO u16 iAdcValue;
extern __IO uint8_t iFdCanFlag;//0 CAN2.0  ;80=FDCAN

extern __IO u16 KwpBaud;//KWP 波特率
extern __IO u8 KwpAdd;//地址码
extern __IO u8 KwpKey1;//key1
extern __IO u8 KwpKey2;//key2
extern __IO u8 KwpValue;//接收到的KWP 值
extern __IO u8 KwpReFlag;//KWP 接收标记 80接收到数据了
extern __IO uint8_t iKwpLastSendValue;//发送的最后一个字节 K线有时候会接收到字节发的最后一个字节 不知道为何

extern __IO uint8_t iUsbRecFlag;//USB 接收数据标记  80接收到上位机 数据

void Led_Init(void);//初始化 普通IO 引脚
void ToggleLed(void);
u8 GetComDatFunEx(u16 CMDcnt);
void Adc_Init(void);
uint16_t adc_channel_sample(uint8_t channel);
void Kline_Init(void);
void SendCan30Buf(void);
///* configure the TIMER peripheral */
//void timer_config(void);
///* configure the TIMER1 interrupt */
//void nvic_config(void);

#endif
