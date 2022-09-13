/*!
    \file    gd32c10x_it.c
    \brief   main interrupt service routines

    \version 2020-12-31, V1.0.0, firmware for GD32C10x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc. 

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32c10x_it.h"
#include "drv_usbd_int.h"
#include "bsp.h"
#include "timer.h"
//#include "vpwm.h"

extern usb_core_driver cdc_acm;
extern uint32_t usbfs_prescaler;
extern void usb_timer_irq (void);

static u8 Old_val1=0x80;//电平状态  0x80  未知   0 低电平  1高电平
//static u32 Oldtmp=0;//上一次电平时间 us

void TIMER0_Channel_IRQnHandler(void)
{
	
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
		}
	
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);
		}
}

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
}

/*!
    \brief      this function handles timer2 interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

/*!
    \brief      this function handles USBFS interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void  USBFS_IRQHandler (void)
{
    usbd_isr (&cdc_acm);
}

/*!
    \brief      this function handles USBFS wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_WKUP_IRQHandler(void)
{
    if (cdc_acm.bp.low_power) {

        SystemInit();

        rcu_usb_clock_config(usbfs_prescaler);

        rcu_periph_clock_enable(RCU_USBFS);

        usb_clock_active(&cdc_acm);
    }

    exti_interrupt_flag_clear(EXTI_18);
}



 /*!
	定时器5溢出中断 用于定时
*/
void TIMER5_IRQHandler(void)
{
	  if(SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP))			
		{
			timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);//清中断标志
			timer_counter_value_config(TIMER5,0);
			iTime5Cmt+=0xFFFE;//溢出值是这个
		}
}

 /*!
	定时器5溢出中断 用于测量
*/
void TIMER6_IRQHandler(void)
{
	  if(SET == timer_interrupt_flag_get(TIMER6, TIMER_INT_FLAG_UP))			
		{
			timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);//清中断标志
			timer_counter_value_config(TIMER6,0);
			iTime6Cmt+=0xFFFE;//溢出值是这个
		}
}



//获取到FD 帧 长度
u8 GetFdCanLenEx(u8 iLen)
{
	u8 iSum=iLen;

	if(iLen==12) iSum=9;
	else if(iLen==16) iSum=10;
	else if(iLen==20) iSum=11;
	else if(iLen==24) iSum=12;
	else if(iLen==32) iSum=13;
	else if(iLen==48) iSum=14;
	else if(iLen==64) iSum=15;
	return iSum;
}

/*!
    \brief      this function handles CAN0 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN0_RX0_IRQHandler(void)
{
//		u32 tmp=0;
//	  u8 j=0,iReCountEx=0,iFdLen=0;
//		u8 iDataTemp[200]={0};
	can_receive_message_struct RbufMege;
		RbufMege.rx_sfid=0;
    /* check the receive message */
  can_message_receive(CAN0, CAN_FIFO0, &RbufMege);
   // can0_receive_flag = SET; 
//	iCan30flag=0x00;
//	iCan30Sum=0;
//	if(iSendComValue==0x8055&&RbufMege.rx_data[0]==0x30)
//	{//模拟ECU 时 发了10帧 需要等设备回复30帧
//		iCan30flag=0x80;
//		iCan30Sum=RbufMege.rx_data[1];//允许发多帧的帧数 
//		//return;
//	}
//		
//	if((iEcuFlag==0x80)&&(RbufMege.rx_data[0]&0xB0)==0xB0)
//	{
//		return ;//TP 2.0 协议不上传 确认帧的
//	}
//		
//		
//	tmp=GetTimer6Cnt();//定时器6 1us 计数器
//	//数据处理
//		if(RbufMege.rx_ff==CAN_FF_STANDARD)
//		{
//			RbufMege.rx_sfid=RbufMege.rx_sfid*0x20;//标准帧
//			iFdLen=GetFdCanLenEx(RbufMege.rx_dlen);
//			iDataTemp[iReCountEx++]=RbufMege.rx_dlen;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_sfid&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_sfid&0x0FF);
//			//id=RbufMege.rx_sfid*0x20;
//		}	
//		else
//		{
//			RbufMege.rx_efid=RbufMege.rx_efid*0x08;//扩展
//			iFdLen=GetFdCanLenEx(RbufMege.rx_dlen);
//			iDataTemp[iReCountEx++]=iFdLen|0x80;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_efid&0x0FF000000)>>24;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_efid&0x0FF0000)>>16;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_efid&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_efid&0x0FF);
//			//id=RbufMege.rx_efid*0x08;
//		}
//	
//		for(j=0;j<RbufMege.rx_dlen;j++)
//		{
//			iDataTemp[iReCountEx++]=RbufMege.rx_data[j];
//		}
//	
//	//	if(iTimeFlag==0x80&&iEcuSimulatorFlag==0)//带时间标志 而且 不能是 模拟器模式
//		if(iEcuFlag!=0x80)//带时间标志 而且 不能是 模拟器模式
//		{
//			//带时间接收
//			iDataTemp[iReCountEx++]=0x3b;
//			iDataTemp[iReCountEx++]=0x54;
//			iDataTemp[iReCountEx++]=0x69;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			
//		}
//		iDataTemp[iReCountEx++]=0x90;//标志
//		iDataTemp[iReCountEx++]='\r';
//		iDataTemp[iReCountEx++]='\n';
//		
//				
//		for(j=0;j<iReCountEx;j++)//存一级缓存
//		{
//			if(iReCount>(4096*4))break;//超了						
//			BUF_MOCAN[iReCount++]=iDataTemp[j];
//		}
		
}

/*!
    \brief      this function handles CAN1 RX0 exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void CAN1_RX0_IRQHandler(void)
{
	  can_receive_message_struct g_receive_message;
    /* check the receive message */
    can_message_receive(CAN1, CAN_FIFO0, &g_receive_message);
   // can1_receive_flag = SET; 
}


/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART1_IRQHandler(void)
{
//	 uint16_t iValue=0;
//		u8 iDataTemp[20]={0},j=0;
//	u32 tmp=0;
//	u8 iReCountEx=0;	
//	tmp=GetTimer6Cnt();//定时器6 1us 计数器
//		KwpReFlag=0;
//    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
//		{
//				
//        /* receive data */
//        iValue = usart_data_receive(USART1);
//				KwpValue=iValue;//全局
//				KwpReFlag=0x80;//接收到数据
//				//usart_flag_clear(USART1,USART_FLAG_RBNE);
//				//iTimeFlag=BUF_MO[8];//时间参数 0 帧间隔，1字节间隔 2 不带时间接收
//			if((tmp>(1000*iKwpIndexTime))&&(iTimeFlag==0)&&iEcuFlag==0x00)//帧间隔		 非模拟ECU 
//			{
//				//不带时间接收
//				iDataTemp[iReCountEx++]=0x90;//标志
//				iDataTemp[iReCountEx++]='\r';
//				iDataTemp[iReCountEx++]='\n';
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*还可以让时间更精确一点*/
//			}
//			else if(iTimeFlag==1)//字节间隔 每个字节都计算时间
//			{
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*还可以让时间更精确一点*/
//				iDataTemp[iReCountEx++]=0x3b;
//				iDataTemp[iReCountEx++]=0x54;
//				iDataTemp[iReCountEx++]=0x69;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			
//				iDataTemp[iReCountEx++]=0x90;//标志
//				iDataTemp[iReCountEx++]='\r';
//				iDataTemp[iReCountEx++]='\n';
//			}
//			else
//			{
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*还可以让时间更精确一点*/
//			}
//			if(iKDataMode==40)  iReCountEx=0;//等待地址码 初始化过程中不要 计算
//			
//			//
////			if(iReCountEx==1)
////			{//iKwpLastSendValue
////				if(iValue==iKwpLastSendValue)//接收到了字节发送的最后一个字节
////				{
////					if((iValue&0x80)!=0x80||(iValue&0xC0)!=0xC0)//
////					{
////						 iReCountEx=0;
////						iKwpLastSendValue=0;
////					}
////				}
////			}

////			
//		for(j=0;j<iReCountEx;j++)//存一级缓存
//		{
//			if(iReCount>(4096*4))break;//超了						
//			BUF_MOCAN[iReCount++]=iDataTemp[j];
//		}
//			
////        if(rxcount == rx_size)
////				{
////            usart_interrupt_disable(USART1, USART_INT_RBNE);
////        }
//    }
////	if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_TBE)){
////			/* transmit data */
////			usart_data_transmit(USART0, txbuffer[txcount++]);
////			if(txcount == tx_size){
////					usart_interrupt_disable(USART0, USART_INT_TBE);
////			}
////	}
}


/*!
    \brief      this function handles TIMER2 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none

PWM_IN   PA0    TIMER1_CH0
VPW_IN    PA1   TIMER1_CH1

VPW协议初始电平为0V，
在第1帧数据前有一个163至239微秒（us）的高电平表示SOF（帧头即数据开始标志），、
接下来以不同长短的高低电平表示二进制数据0或1，其中：
高电平宽度介于34-96us表示“1”，高电平宽度介于96-163us表示“0”， 
低电平宽度介于34-96us表示“0”，低电平宽度介于96-163us表示“1”，
68 6A F1 01 00 17    字节之间无间隔

传输时按字节顺序，且每个字节都是高位在前，低位在后的顺序，
高低电平相间用于表示传输的数据，字节与字节之间没有间隔，
传送完一帧数据之后有一个宽度大于239us的低电平表示EOF（帧尾即帧结束标志）。

01 ;Times 200 us     SOF

;68 电平如下   0110 1000
00 ;Times 61 us     0
01 ;Times 63 us			1
00 ;Times 126 us    1
01 ;Times 127 us    0
00 ;Times 126 us    1
01 ;Times 127 us    0
00 ;Times 62 us     0
01 ;Times 127 us    0

PWM 说明
初始是低电平
PWM  是高低电平组合 成 BIT0   1,高位在前 低位在后
开始 会有 
01 ;Times 30 us
00 ;Times 13 us     SOF  
//61 电平如下    0110 0001
01 ;Times 14 us      0
00 ;Times 6 us 

01 ;Times 7 us       1 
00 ;Times 15 us

01 ;Times 7 us       1
00 ;Times 15 us

01 ;Times 15 us      0
00 ;Times 6 us

01 ;Times 15 us      0
00 ;Times 6 us

01 ;Times 15 us      0
00 ;Times 6 us

01 ;Times 15 us      0
00 ;Times 6 us

01 ;Times 7 us       1
00 ;Times 14 us

*/
void TIMER1_IRQHandler(void)
{
		u32 tmp=0,j=0;
		u8 iDataTemp[10]={0};
		u8 iReCountEx=0;
		u8 key_val1=0;//电平状态
		u8 EofFlag=0;//E0F
		//BOOL flag=true;
		
		//1*65535=65.535ms产生一次溢出中断
		if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//保存溢出中断的时间
			timer_counter_value_config(TIMER1, 0);//清空计数器
			//TIM_SetCounter(TIM5,0);//清空计数器
			return ;
		}		
		
		iJ1850EofFlag=0;//帧清空标记
		
		
		//timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);//清中断过标志
		//return ;
		
		//边沿变化
		//当边沿变化时 产生中断,此时获取计数器 即可获得上一段电平变化的时间
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH0))//PWM
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);//清中断过标志
			
				tmp = timer_channel_capture_value_register_read(TIMER1, TIMER_CH_0);//读取计数器值 
				timer_counter_value_config(TIMER1, 0);//清空计数器
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
//				if(tmp%10)
//				{
//					tmp+=10;
//				}
//				tmp=tmp/10;
				
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))//低电平
				{
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_0,TIMER_IC_POLARITY_RISING);//下一个时刻会是上升沿 
					  key_val1=1;
         }
				else//高电平 表示下个时刻是 低电平
				{
						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_0,TIMER_IC_POLARITY_FALLING);//下降沿
				}
			
				if(Old_val1!=0x80) //初始电平 状态
				{
					//if(Old_val1==key_val1) //与上一次电平相同了 不要
						//return ;
				}
					
				Old_val1=key_val1;
				
//				if(iJ1850BitSum>900)iJ1850BitSum=0;//避免越界了
//				//SOF（帧起始标志）时间，常规为48us，取值范围，发送时47<=TP4<=51,接收时46<=TP4<=63 
//				//EOF（帧结束标志）时间，常规为72us，取值范围，发送时70<=TP5<=76.5，接收时TP5>=70

//				if(tmp<45&&tmp>4)	//小于4us 的电平不要				
//				 BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
				if(tmp<48&&tmp>2&&iTimeFlag==0)	//小于4us的电平不要		
				{					
					//BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
					if(key_val1==1)
						j1850_Hbit[iHBitSum++]=(u8)tmp;
					else 
						j1850_Lbit[iLBitSum++]=(u8)tmp;					
					
					if(iHBitSum==1&&iLBitSum==1)
					{//帧头有一帧是 30us高电平的信号
						if(j1850_Hbit[0]>24)
						{
							iHBitSum=0;
							iLBitSum=0;
							//SOF	
							if(iEcuFlag==0x00)
							{
								BUF_MOCAN[iReCount++]=0x90;//标志
								BUF_MOCAN[iReCount++]='\r';
								BUF_MOCAN[iReCount++]='\n';
							}
					
						}
					}
					
					//pwm是高低电平表示一个BIT
					if(iLBitSum==8&&iHBitSum==8&&iPartValue==7)//一个字节了
					{
					//BUF_MOCAN[iReCount++]=GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						iHBitSum=0;//
					  iLBitSum=0;//
					}
				}
				else if(tmp>48&&iTimeFlag==0)//EOF SOF
				{

					//pwm是高低电平表示一个BIT  EOF 是低电平
					if(iLBitSum==7&&iHBitSum==8&&iPartValue==7)//一个字节了
					{//最后一个电平是低电平 和 EOF 连在一起了
							if(j1850_Hbit[7]>10) j1850_Lbit[iLBitSum++]=8;		
							else 		j1850_Lbit[iLBitSum++]=16;
						//不懂为何模拟平台有 会有个F1 
						//tmp=GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						if(tmp!=0xf1&&iEcuFlag==0x00)  
						{	
							BUF_MOCAN[iReCount++]=tmp;//GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						}
					}

						EofFlag=1;//EOF 标记
						iHBitSum=0;//
						iLBitSum=0;//
				}
    }
		 //边沿变化		//当边沿变化时 产生中断,此时获取计数器 即可获得上一段电平变化的时间
    else if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH1))//VPW
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH1);//清中断过标志
			
				tmp = timer_channel_capture_value_register_read(TIMER1, TIMER_CH_1);//读取计数器值 
				timer_counter_value_config(TIMER1, 0);//清空计数器
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//低电平
				{
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//下一个时刻会是上升沿 
					  key_val1=1;
         }
				else//高电平 表示下个时刻是 低电平
				{
						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//下降沿
				}
				
				if(Old_val1!=0x80) //初始电平 状态
				{
					if(Old_val1==key_val1) //与上一次电平相同了 不要
						return ;
				}
					
				Old_val1=key_val1;
				
				//if(iJ1850BitSum>900)iJ1850BitSum=0;//避免越界了
				
				//163至239微秒（us）的     SOF
				//宽度大于239us的低电平表示EOF
				if(tmp<160&&tmp>30&&iTimeFlag==0)	//小于30us的电平不要		
				{					
					//BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
					if(key_val1==1)
						j1850_Hbit[iHBitSum++]=(u8)tmp;
					else 
						j1850_Lbit[iLBitSum++]=(u8)tmp;					
					
					if(iLBitSum==4&&iHBitSum==4&&iPartValue==6)//一个字节了
					{
						//BUF_MOCAN[iReCount++]=GetVpwBitToByte(j1850_Hbit,j1850_Lbit);
						iHBitSum=0;//
					  iLBitSum=0;//
					}
				}
				else if(tmp>160&&iTimeFlag==0)
				{//大于239us的低电平表示EOF
					//一个163至239微秒（us）的高电平表示SOF
					//iEcuFlag=0 表示是采集数据
					if((tmp>163&&tmp<239)&&(key_val1==1)&&(iEcuFlag==0x00)) //SOF
					{
							BUF_MOCAN[iReCount++]=0x90;//标志
							BUF_MOCAN[iReCount++]='\r';
							BUF_MOCAN[iReCount++]='\n';
							iJ1850EofFlag=1;
					}
					else if(tmp>239&&key_val1==0) //EOF
					{
						EofFlag=1;//EOF 标记
					}
//					Oldtmp=tmp;
					iHBitSum=0;//
					iLBitSum=0;//
				}
    }
		
		
		
		//电平宽度的 
		if(iTimeFlag==1)
		{
			iDataTemp[iReCountEx++]=key_val1;//高低电平状态			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;

			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//标志
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';			
			for(j=0;j<iReCountEx;j++)//存一级缓存
			{
				if(iReCount>(4096*4))break;//超了						
				BUF_MOCAN[iReCount++]=iDataTemp[j];
			}
	 }
	 else if(EofFlag==1)//E0F
	 {
//			BUF_MOCAN[iReCount++]=0x3b;
//			BUF_MOCAN[iReCount++]=0x54;
//			BUF_MOCAN[iReCount++]=0x69;

//			BUF_MOCAN[iReCount++]=(tmp&0x0FF0000)>>16;
//			BUF_MOCAN[iReCount++]=(tmp&0x0FF00)>>8;
//			BUF_MOCAN[iReCount++]=(tmp&0x0FF);
//			BUF_MOCAN[iReCount++]=0x90;//标志
//			BUF_MOCAN[iReCount++]='\r';
//			BUF_MOCAN[iReCount++]='\n';			
	 }		 
}




/*!
    \brief      this function handles TIMER2 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
//time0 8 共用 中断
//		  TIMER0_BRK_TIMER8_IRQn
void TIMER0_BRK_TIMER8_IRQHandler(void)
{
		u32 tmp=0,j=0;
		u8 iDataTemp[10]={0};
		u8 iReCountEx=0;
		u8 key_val1=0;//电平状态
		
		//1*65535=65.535ms产生一次溢出中断
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//保存溢出中断的时间
			timer_counter_value_config(TIMER0, 0);//清空计数器
			//TIM_SetCounter(TIM5,0);//清空计数器
			return ;
		}		
		//1*65535=65.535ms产生一次溢出中断
		else if(SET == timer_interrupt_flag_get(TIMER8, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//保存溢出中断的时间
			timer_counter_value_config(TIMER8, 0);//清空计数器
			//TIM_SetCounter(TIM5,0);//清空计数器
			return ;
		}		
		
		//TIME0 只有单边捕获
		//边沿变化
		//当边沿变化时 产生中断,此时获取计数器 即可获得上一段电平变化的时间
    if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2))
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);//清中断过标志
			
				tmp = timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2);//读取计数器值 
				timer_counter_value_config(TIMER0, 0);//清空计数器
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//低电平
				{
						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//下一个时刻会是上升沿 
					  key_val1=1;
         }
				else//高电平 表示下个时刻是 低电平
				{
						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0
						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//下降沿
						
				}
			
    }
		//边沿变化
		//当边沿变化时 产生中断,此时获取计数器 即可获得上一段电平变化的时间
    else if(SET == timer_interrupt_flag_get(TIMER8, TIMER_INT_FLAG_CH1))
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_CH1);//清中断过标志
			
				tmp = timer_channel_capture_value_register_read(TIMER8, TIMER_CH_1);//读取计数器值 
				timer_counter_value_config(TIMER8, 0);//清空计数器
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_3))//低电平
				{
						//timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//下一个时刻会是上升沿 
					  key_val1=1;
         }
				else//高电平 表示下个时刻是 低电平
				{
						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0
					//	timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//下降沿
						
				}
			
    }
		
			iDataTemp[iReCountEx++]=key_val1;//高低电平状态
			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;

			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//标志
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';
			
		for(j=0;j<iReCountEx;j++)//存一级缓存
		{
			if(iReCount>(4096*4))break;//超了						
			BUF_MOCAN[iReCount++]=iDataTemp[j];
		}
		
		
}


//time0 计数中断
//		  
void TIMER0_UP_TIMER9_IRQnHandler(void)
{
//		/*u32 tmp=0,j=0*/;
//		u8 iDataTemp[10]={0};
//		u8 iReCountEx=0;
//		u8 key_val1=0;//电平状态
		
		//1*65535=65.535ms产生一次溢出中断
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//保存溢出中断的时间
			timer_counter_value_config(TIMER0, 0);//清空计数器
			//TIM_SetCounter(TIM5,0);//清空计数器
			return ;
		}		
	
//		
//		//TIME0 只有单边捕获
//		//边沿变化
//		//当边沿变化时 产生中断,此时获取计数器 即可获得上一段电平变化的时间
//    if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2))
//		{
//        /* clear channel 0 interrupt bit */
//        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);//清中断过标志
//			
//				tmp = timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2);//读取计数器值 
//				timer_counter_value_config(TIMER0, 0);//清空计数器
//			
//				iTim1Value+=tmp;
//				tmp=iTim1Value;
//				iTim1Value=0;
//			
//			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//低电平
//				{
//						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//下一个时刻会是上升沿 
//					  key_val1=1;
//         }
//				else//高电平 表示下个时刻是 低电平
//				{
//						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0
//						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//下降沿
//						
//				}
//			
//    }
//	
//			iDataTemp[iReCountEx++]=key_val1;//高低电平状态
//			
//			iDataTemp[iReCountEx++]=0x3b;
//			iDataTemp[iReCountEx++]=0x54;
//			iDataTemp[iReCountEx++]=0x69;

//			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			iDataTemp[iReCountEx++]=0x90;//标志
//			iDataTemp[iReCountEx++]='\r';
//			iDataTemp[iReCountEx++]='\n';
//			
//		for(j=0;j<iReCountEx;j++)//存一级缓存
//		{
//			if(iReCount>(4096*4))break;//超了						
//			BUF_MOCAN[iReCount++]=iDataTemp[j];
//		}
		
		
}


/*!
    \brief      this function handles external lines 10 to 15 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
外部中断 是一个周期的电平变化 并不能检测 电平翻转时间

*/
void EXTI10_15_IRQHandler(void)
{
	
			u32 tmp=0,j=0;
		u8 iDataTemp[10]={0};
		u8 iReCountEx=0;
		u8 key_val1=0;//电平状态
    /* check the key tamper is pressed or not */
    if (RESET != exti_interrupt_flag_get(EXTI_10))
		{
        //gpio_bit_write(GPIOA, GPIO_PIN_10, (bit_status)(1-gpio_input_bit_get(GPIOE, GPIO_PIN_0)));
				tmp=GetTimer6Cnt();//定时器6 1us 计数器;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//低电平
				{
					  key_val1=1;
         }
				else//高电平 表示下个时刻是 低电平
				{
						key_val1=0;//现在是高电平 表明 上一次是 低电平 所以 是0						
				}
			
        exti_interrupt_flag_clear(EXTI_10);
    }

			iDataTemp[iReCountEx++]=key_val1;//高低电平状态			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;
			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//标志
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';
			
		for(j=0;j<iReCountEx;j++)//存一级缓存
		{
			if(iReCount>(4096*4))break;//超了						
			BUF_MOCAN[iReCount++]=iDataTemp[j];
		}
		
		GetTimer6Cnt();//定时器6 1us 计数器  清空一下
}

