#ifndef __BSP_H
#define __BSP_H	 
#include "gd32C10x.h"
#include "string.h"
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

extern __IO uint32_t  iTimeCmt;
extern __IO uint32_t  iTime5Cmt;   //��ʱ��5������
extern __IO uint32_t   iTime6Cmt;   //��ʱ��6������
extern __IO uint32_t 			iTim1Value;//��ʱ��1 ���벶�����ֵ


extern  u8 BUF_MO[1024*4];
extern  u8 BUF_MOCAN[1024*4];
extern  u8 ReDataStr[1024*7];//���ջ�����


extern u8 j1850_Hbit[500];//J1850   �ߵ�ƽʱ��
extern u8 j1850_Lbit[500];//J1850   �͵�ƽʱ��

extern __IO u16 iHBitSum;//�ߵ�ƽ����
extern __IO u16 iLBitSum;//�͵�ƽ ����
extern __IO u8 iPartValue;

extern  __IO u32 iReCount;//������֡��		
extern  __IO u8  iKDataMode;
extern  __IO u8  iTimeFlag;
extern __IO uint8_t  iKwpIndexTime;//KWP ֡���ʱ��
extern __IO u8 iJ1850EofFlag;//1850 ֡�������   1=֡����  

extern __IO u8 iEcuFlag;//�Ƿ���ģ��ECU  00�ɼ� ���� 80 ECU ģ��
extern __IO u16 iSendComValue;
extern __IO u16 iCanSendSum ;

extern __IO u8 Fd_Working_mode; //����ģʽ


extern __IO u8 iCan30flag;//���յ�30֡ ��� 00=��Ҫ�ȴ�30֡��80=�Ѿ����յ�30֡ 
extern __IO u8 iCan30Sum;//30֡�������͵Ķ�֡ ֡��
extern __IO u16 iAdcValue;
extern __IO uint8_t iFdCanFlag;//0 CAN2.0  ;80=FDCAN

extern __IO u16 KwpBaud;//KWP ������
extern __IO u8 KwpAdd;//��ַ��
extern __IO u8 KwpKey1;//key1
extern __IO u8 KwpKey2;//key2
extern __IO u8 KwpValue;//���յ���KWP ֵ
extern __IO u8 KwpReFlag;//KWP ���ձ�� 80���յ�������
extern __IO uint8_t iKwpLastSendValue;//���͵����һ���ֽ� K����ʱ�����յ��ֽڷ������һ���ֽ� ��֪��Ϊ��

extern __IO uint8_t iUsbRecFlag;//USB �������ݱ��  80���յ���λ�� ����

void Led_Init(void);//��ʼ�� ��ͨIO ����
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
