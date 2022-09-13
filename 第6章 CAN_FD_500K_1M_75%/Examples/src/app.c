

/*
	CAN FD 500K_2M 73% ���ԣ�OBD6 CANH,OBD14 CANL
  
	ע��:120R*2 �ն˵�����Ҫ����
	siyuan 2022-09-06
*/

#include "drv_usb_hw.h"
#include "cdc_acm_core.h"

#include	<stdio.h>
#include "can.h"
#include "bsp.h"
#include "timer.h"
//#include "usart.h"	 
//#include "vpwm.h"

usb_core_driver cdc_acm;

extern  uint16_t iRxUsbLen;//����Usb���ݳ���
extern uint16_t iRxUsbFlag;//������ɱ��  0x80�������

void GetKLineTime(void);

void EcuSendData(void);
u8 RecAdd(void);
u8 Wait5BpsAdd(void);
/*!
    \brief      main routine will construct a USB mass storage device
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
		uint8_t SendData[10]={0x08,0X02,0X10,0x03,0x00,0x00,0x00,0x00,0x00};

		uint8_t SendData1[100]={0x09,0X02,0X10,0x03,0x00,0x00,0x00,0x00,0x00};
		
//		uint8_t SendData2[100]={0x0F,0X02,0X10,0x01,0x02,0x03,0x04,0x05,0x00};
		uint8_t i=0;
		iRxUsbLen=0;
		iRxUsbFlag=0;
		
    usb_rcu_config();

    usb_timer_init();

    usbd_init (&cdc_acm, USB_CORE_ENUM_FS, &cdc_desc, &cdc_class);

    usb_intr_config();
    
#ifdef USE_IRC48M
    /* CTC peripheral clock enable */
    rcu_periph_clock_enable(RCU_CTC);

    /* CTC configure */
    ctc_config();

    while (ctc_flag_get(CTC_FLAG_CKOK) == RESET) {
    }
#endif

		//
	//StCanInitTest();//CAN test
	//	FdCanInitTest();//FD CAN
		
		TIM5_config();//���ڶ�ʱ
		TIM6_config();//���ڲ���ʱ��
		Led_Init();//��ʼ�� ��ͨIO

		
		iHBitSum=0;//
		iLBitSum=0;//
		//��ʼ������
		iKDataMode=0;
		iPartValue=0;
		iEcuFlag=0;//����ģʽ
		
		Adc_Init();//��ʼ����ѹ����
		
		CanFD_config(can_500k,Data_1M);//CAN FD 500k  4M 80%
		//CAN1_Config16BitFilter(0xFC00,0xFD00);//���ù���ID 
		CAN_setAllfit();//���ò�����ID
		
    /* main loop */
  while (1) 
	{
		
		CanFdSendISO15765Data(SendData,0xfc00);//15765 STCAN
		Delay_ms(100);
		CanFdSendISO15765Data(SendData,0x18DB33F1);//15765 EXCAN 
		Delay_ms(100);
		
		for(i=0;i<0x10;i++)
		{
			SendData1[0]=i;//���ȸı�
			SendData1[1]=i;
			CanFdSendISO15765Data(SendData1,0xfd00);//15765 STCAN �䳤�� ��֤CAN FD 
			Delay_ms(100);
		}
		//TEST
	 if (USBD_CONFIGURED == cdc_acm.dev.cur_status) 
	 {
      if (0U == cdc_acm_check_ready(&cdc_acm)) 
			{
         cdc_acm_data_receive(&cdc_acm);		
					// ׼���������� 							           
			}
			else 
			{							//��������
        //cdc_acm_data_send(&cdc_acm);
      }
		} 
		Delay_us(10);
		continue;				
	
    }
}



