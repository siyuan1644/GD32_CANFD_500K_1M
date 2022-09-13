

/*
	CAN FD 500K_2M 73% 测试，OBD6 CANH,OBD14 CANL
  
	注意:120R*2 终端电阻需要加上
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

extern  uint16_t iRxUsbLen;//接收Usb数据长度
extern uint16_t iRxUsbFlag;//接收完成标记  0x80接收完成

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
		
		TIM5_config();//用于定时
		TIM6_config();//用于测量时间
		Led_Init();//初始化 普通IO

		
		iHBitSum=0;//
		iLBitSum=0;//
		//初始化变量
		iKDataMode=0;
		iPartValue=0;
		iEcuFlag=0;//采数模式
		
		Adc_Init();//初始化电压采样
		
		CanFD_config(can_500k,Data_1M);//CAN FD 500k  4M 80%
		//CAN1_Config16BitFilter(0xFC00,0xFD00);//设置过滤ID 
		CAN_setAllfit();//设置不过滤ID
		
    /* main loop */
  while (1) 
	{
		
		CanFdSendISO15765Data(SendData,0xfc00);//15765 STCAN
		Delay_ms(100);
		CanFdSendISO15765Data(SendData,0x18DB33F1);//15765 EXCAN 
		Delay_ms(100);
		
		for(i=0;i<0x10;i++)
		{
			SendData1[0]=i;//长度改变
			SendData1[1]=i;
			CanFdSendISO15765Data(SendData1,0xfd00);//15765 STCAN 变长度 验证CAN FD 
			Delay_ms(100);
		}
		//TEST
	 if (USBD_CONFIGURED == cdc_acm.dev.cur_status) 
	 {
      if (0U == cdc_acm_check_ready(&cdc_acm)) 
			{
         cdc_acm_data_receive(&cdc_acm);		
					// 准备接收数据 							           
			}
			else 
			{							//发送数据
        //cdc_acm_data_send(&cdc_acm);
      }
		} 
		Delay_us(10);
		continue;				
	
    }
}



