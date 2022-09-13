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

static u8 Old_val1=0x80;//��ƽ״̬  0x80  δ֪   0 �͵�ƽ  1�ߵ�ƽ
//static u32 Oldtmp=0;//��һ�ε�ƽʱ�� us

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
	��ʱ��5����ж� ���ڶ�ʱ
*/
void TIMER5_IRQHandler(void)
{
	  if(SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP))			
		{
			timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);//���жϱ�־
			timer_counter_value_config(TIMER5,0);
			iTime5Cmt+=0xFFFE;//���ֵ�����
		}
}

 /*!
	��ʱ��5����ж� ���ڲ���
*/
void TIMER6_IRQHandler(void)
{
	  if(SET == timer_interrupt_flag_get(TIMER6, TIMER_INT_FLAG_UP))			
		{
			timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);//���жϱ�־
			timer_counter_value_config(TIMER6,0);
			iTime6Cmt+=0xFFFE;//���ֵ�����
		}
}



//��ȡ��FD ֡ ����
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
//	{//ģ��ECU ʱ ����10֡ ��Ҫ���豸�ظ�30֡
//		iCan30flag=0x80;
//		iCan30Sum=RbufMege.rx_data[1];//������֡��֡�� 
//		//return;
//	}
//		
//	if((iEcuFlag==0x80)&&(RbufMege.rx_data[0]&0xB0)==0xB0)
//	{
//		return ;//TP 2.0 Э�鲻�ϴ� ȷ��֡��
//	}
//		
//		
//	tmp=GetTimer6Cnt();//��ʱ��6 1us ������
//	//���ݴ���
//		if(RbufMege.rx_ff==CAN_FF_STANDARD)
//		{
//			RbufMege.rx_sfid=RbufMege.rx_sfid*0x20;//��׼֡
//			iFdLen=GetFdCanLenEx(RbufMege.rx_dlen);
//			iDataTemp[iReCountEx++]=RbufMege.rx_dlen;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_sfid&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(RbufMege.rx_sfid&0x0FF);
//			//id=RbufMege.rx_sfid*0x20;
//		}	
//		else
//		{
//			RbufMege.rx_efid=RbufMege.rx_efid*0x08;//��չ
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
//	//	if(iTimeFlag==0x80&&iEcuSimulatorFlag==0)//��ʱ���־ ���� ������ ģ����ģʽ
//		if(iEcuFlag!=0x80)//��ʱ���־ ���� ������ ģ����ģʽ
//		{
//			//��ʱ�����
//			iDataTemp[iReCountEx++]=0x3b;
//			iDataTemp[iReCountEx++]=0x54;
//			iDataTemp[iReCountEx++]=0x69;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			
//		}
//		iDataTemp[iReCountEx++]=0x90;//��־
//		iDataTemp[iReCountEx++]='\r';
//		iDataTemp[iReCountEx++]='\n';
//		
//				
//		for(j=0;j<iReCountEx;j++)//��һ������
//		{
//			if(iReCount>(4096*4))break;//����						
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
//	tmp=GetTimer6Cnt();//��ʱ��6 1us ������
//		KwpReFlag=0;
//    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
//		{
//				
//        /* receive data */
//        iValue = usart_data_receive(USART1);
//				KwpValue=iValue;//ȫ��
//				KwpReFlag=0x80;//���յ�����
//				//usart_flag_clear(USART1,USART_FLAG_RBNE);
//				//iTimeFlag=BUF_MO[8];//ʱ����� 0 ֡�����1�ֽڼ�� 2 ����ʱ�����
//			if((tmp>(1000*iKwpIndexTime))&&(iTimeFlag==0)&&iEcuFlag==0x00)//֡���		 ��ģ��ECU 
//			{
//				//����ʱ�����
//				iDataTemp[iReCountEx++]=0x90;//��־
//				iDataTemp[iReCountEx++]='\r';
//				iDataTemp[iReCountEx++]='\n';
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*��������ʱ�����ȷһ��*/
//			}
//			else if(iTimeFlag==1)//�ֽڼ�� ÿ���ֽڶ�����ʱ��
//			{
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*��������ʱ�����ȷһ��*/
//				iDataTemp[iReCountEx++]=0x3b;
//				iDataTemp[iReCountEx++]=0x54;
//				iDataTemp[iReCountEx++]=0x69;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//				iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			
//				iDataTemp[iReCountEx++]=0x90;//��־
//				iDataTemp[iReCountEx++]='\r';
//				iDataTemp[iReCountEx++]='\n';
//			}
//			else
//			{
//				iDataTemp[iReCountEx++]=iValue;//USART_ReceiveData(USART2);/*��������ʱ�����ȷһ��*/
//			}
//			if(iKDataMode==40)  iReCountEx=0;//�ȴ���ַ�� ��ʼ�������в�Ҫ ����
//			
//			//
////			if(iReCountEx==1)
////			{//iKwpLastSendValue
////				if(iValue==iKwpLastSendValue)//���յ����ֽڷ��͵����һ���ֽ�
////				{
////					if((iValue&0x80)!=0x80||(iValue&0xC0)!=0xC0)//
////					{
////						 iReCountEx=0;
////						iKwpLastSendValue=0;
////					}
////				}
////			}

////			
//		for(j=0;j<iReCountEx;j++)//��һ������
//		{
//			if(iReCount>(4096*4))break;//����						
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

VPWЭ���ʼ��ƽΪ0V��
�ڵ�1֡����ǰ��һ��163��239΢�루us���ĸߵ�ƽ��ʾSOF��֡ͷ�����ݿ�ʼ��־������
�������Բ�ͬ���̵ĸߵ͵�ƽ��ʾ����������0��1�����У�
�ߵ�ƽ��Ƚ���34-96us��ʾ��1�����ߵ�ƽ��Ƚ���96-163us��ʾ��0���� 
�͵�ƽ��Ƚ���34-96us��ʾ��0�����͵�ƽ��Ƚ���96-163us��ʾ��1����
68 6A F1 01 00 17    �ֽ�֮���޼��

����ʱ���ֽ�˳����ÿ���ֽڶ��Ǹ�λ��ǰ����λ�ں��˳��
�ߵ͵�ƽ������ڱ�ʾ��������ݣ��ֽ����ֽ�֮��û�м����
������һ֡����֮����һ����ȴ���239us�ĵ͵�ƽ��ʾEOF��֡β��֡������־����

01 ;Times 200 us     SOF

;68 ��ƽ����   0110 1000
00 ;Times 61 us     0
01 ;Times 63 us			1
00 ;Times 126 us    1
01 ;Times 127 us    0
00 ;Times 126 us    1
01 ;Times 127 us    0
00 ;Times 62 us     0
01 ;Times 127 us    0

PWM ˵��
��ʼ�ǵ͵�ƽ
PWM  �Ǹߵ͵�ƽ��� �� BIT0   1,��λ��ǰ ��λ�ں�
��ʼ ���� 
01 ;Times 30 us
00 ;Times 13 us     SOF  
//61 ��ƽ����    0110 0001
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
		u8 key_val1=0;//��ƽ״̬
		u8 EofFlag=0;//E0F
		//BOOL flag=true;
		
		//1*65535=65.535ms����һ������ж�
		if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//��������жϵ�ʱ��
			timer_counter_value_config(TIMER1, 0);//��ռ�����
			//TIM_SetCounter(TIM5,0);//��ռ�����
			return ;
		}		
		
		iJ1850EofFlag=0;//֡��ձ��
		
		
		//timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);//���жϹ���־
		//return ;
		
		//���ر仯
		//�����ر仯ʱ �����ж�,��ʱ��ȡ������ ���ɻ����һ�ε�ƽ�仯��ʱ��
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH0))//PWM
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);//���жϹ���־
			
				tmp = timer_channel_capture_value_register_read(TIMER1, TIMER_CH_0);//��ȡ������ֵ 
				timer_counter_value_config(TIMER1, 0);//��ռ�����
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
//				if(tmp%10)
//				{
//					tmp+=10;
//				}
//				tmp=tmp/10;
				
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_0))//�͵�ƽ
				{
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_0,TIMER_IC_POLARITY_RISING);//��һ��ʱ�̻��������� 
					  key_val1=1;
         }
				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
				{
						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_0,TIMER_IC_POLARITY_FALLING);//�½���
				}
			
				if(Old_val1!=0x80) //��ʼ��ƽ ״̬
				{
					//if(Old_val1==key_val1) //����һ�ε�ƽ��ͬ�� ��Ҫ
						//return ;
				}
					
				Old_val1=key_val1;
				
//				if(iJ1850BitSum>900)iJ1850BitSum=0;//����Խ����
//				//SOF��֡��ʼ��־��ʱ�䣬����Ϊ48us��ȡֵ��Χ������ʱ47<=TP4<=51,����ʱ46<=TP4<=63 
//				//EOF��֡������־��ʱ�䣬����Ϊ72us��ȡֵ��Χ������ʱ70<=TP5<=76.5������ʱTP5>=70

//				if(tmp<45&&tmp>4)	//С��4us �ĵ�ƽ��Ҫ				
//				 BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
				if(tmp<48&&tmp>2&&iTimeFlag==0)	//С��4us�ĵ�ƽ��Ҫ		
				{					
					//BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
					if(key_val1==1)
						j1850_Hbit[iHBitSum++]=(u8)tmp;
					else 
						j1850_Lbit[iLBitSum++]=(u8)tmp;					
					
					if(iHBitSum==1&&iLBitSum==1)
					{//֡ͷ��һ֡�� 30us�ߵ�ƽ���ź�
						if(j1850_Hbit[0]>24)
						{
							iHBitSum=0;
							iLBitSum=0;
							//SOF	
							if(iEcuFlag==0x00)
							{
								BUF_MOCAN[iReCount++]=0x90;//��־
								BUF_MOCAN[iReCount++]='\r';
								BUF_MOCAN[iReCount++]='\n';
							}
					
						}
					}
					
					//pwm�Ǹߵ͵�ƽ��ʾһ��BIT
					if(iLBitSum==8&&iHBitSum==8&&iPartValue==7)//һ���ֽ���
					{
					//BUF_MOCAN[iReCount++]=GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						iHBitSum=0;//
					  iLBitSum=0;//
					}
				}
				else if(tmp>48&&iTimeFlag==0)//EOF SOF
				{

					//pwm�Ǹߵ͵�ƽ��ʾһ��BIT  EOF �ǵ͵�ƽ
					if(iLBitSum==7&&iHBitSum==8&&iPartValue==7)//һ���ֽ���
					{//���һ����ƽ�ǵ͵�ƽ �� EOF ����һ����
							if(j1850_Hbit[7]>10) j1850_Lbit[iLBitSum++]=8;		
							else 		j1850_Lbit[iLBitSum++]=16;
						//����Ϊ��ģ��ƽ̨�� ���и�F1 
						//tmp=GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						if(tmp!=0xf1&&iEcuFlag==0x00)  
						{	
							BUF_MOCAN[iReCount++]=tmp;//GetPWMBitToByte(j1850_Hbit,j1850_Lbit);
						}
					}

						EofFlag=1;//EOF ���
						iHBitSum=0;//
						iLBitSum=0;//
				}
    }
		 //���ر仯		//�����ر仯ʱ �����ж�,��ʱ��ȡ������ ���ɻ����һ�ε�ƽ�仯��ʱ��
    else if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH1))//VPW
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH1);//���жϹ���־
			
				tmp = timer_channel_capture_value_register_read(TIMER1, TIMER_CH_1);//��ȡ������ֵ 
				timer_counter_value_config(TIMER1, 0);//��ռ�����
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_1))//�͵�ƽ
				{
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//��һ��ʱ�̻��������� 
					  key_val1=1;
         }
				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
				{
						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0
						timer_channel_output_polarity_config(TIMER1,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//�½���
				}
				
				if(Old_val1!=0x80) //��ʼ��ƽ ״̬
				{
					if(Old_val1==key_val1) //����һ�ε�ƽ��ͬ�� ��Ҫ
						return ;
				}
					
				Old_val1=key_val1;
				
				//if(iJ1850BitSum>900)iJ1850BitSum=0;//����Խ����
				
				//163��239΢�루us����     SOF
				//��ȴ���239us�ĵ͵�ƽ��ʾEOF
				if(tmp<160&&tmp>30&&iTimeFlag==0)	//С��30us�ĵ�ƽ��Ҫ		
				{					
					//BUF_j1850[iJ1850BitSum++]=key_val1;//VPW
					if(key_val1==1)
						j1850_Hbit[iHBitSum++]=(u8)tmp;
					else 
						j1850_Lbit[iLBitSum++]=(u8)tmp;					
					
					if(iLBitSum==4&&iHBitSum==4&&iPartValue==6)//һ���ֽ���
					{
						//BUF_MOCAN[iReCount++]=GetVpwBitToByte(j1850_Hbit,j1850_Lbit);
						iHBitSum=0;//
					  iLBitSum=0;//
					}
				}
				else if(tmp>160&&iTimeFlag==0)
				{//����239us�ĵ͵�ƽ��ʾEOF
					//һ��163��239΢�루us���ĸߵ�ƽ��ʾSOF
					//iEcuFlag=0 ��ʾ�ǲɼ�����
					if((tmp>163&&tmp<239)&&(key_val1==1)&&(iEcuFlag==0x00)) //SOF
					{
							BUF_MOCAN[iReCount++]=0x90;//��־
							BUF_MOCAN[iReCount++]='\r';
							BUF_MOCAN[iReCount++]='\n';
							iJ1850EofFlag=1;
					}
					else if(tmp>239&&key_val1==0) //EOF
					{
						EofFlag=1;//EOF ���
					}
//					Oldtmp=tmp;
					iHBitSum=0;//
					iLBitSum=0;//
				}
    }
		
		
		
		//��ƽ��ȵ� 
		if(iTimeFlag==1)
		{
			iDataTemp[iReCountEx++]=key_val1;//�ߵ͵�ƽ״̬			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;

			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//��־
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';			
			for(j=0;j<iReCountEx;j++)//��һ������
			{
				if(iReCount>(4096*4))break;//����						
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
//			BUF_MOCAN[iReCount++]=0x90;//��־
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
//time0 8 ���� �ж�
//		  TIMER0_BRK_TIMER8_IRQn
void TIMER0_BRK_TIMER8_IRQHandler(void)
{
		u32 tmp=0,j=0;
		u8 iDataTemp[10]={0};
		u8 iReCountEx=0;
		u8 key_val1=0;//��ƽ״̬
		
		//1*65535=65.535ms����һ������ж�
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//��������жϵ�ʱ��
			timer_counter_value_config(TIMER0, 0);//��ռ�����
			//TIM_SetCounter(TIM5,0);//��ռ�����
			return ;
		}		
		//1*65535=65.535ms����һ������ж�
		else if(SET == timer_interrupt_flag_get(TIMER8, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//��������жϵ�ʱ��
			timer_counter_value_config(TIMER8, 0);//��ռ�����
			//TIM_SetCounter(TIM5,0);//��ռ�����
			return ;
		}		
		
		//TIME0 ֻ�е��߲���
		//���ر仯
		//�����ر仯ʱ �����ж�,��ʱ��ȡ������ ���ɻ����һ�ε�ƽ�仯��ʱ��
    if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2))
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);//���жϹ���־
			
				tmp = timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2);//��ȡ������ֵ 
				timer_counter_value_config(TIMER0, 0);//��ռ�����
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//�͵�ƽ
				{
						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//��һ��ʱ�̻��������� 
					  key_val1=1;
         }
				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
				{
						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0
						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//�½���
						
				}
			
    }
		//���ر仯
		//�����ر仯ʱ �����ж�,��ʱ��ȡ������ ���ɻ����һ�ε�ƽ�仯��ʱ��
    else if(SET == timer_interrupt_flag_get(TIMER8, TIMER_INT_FLAG_CH1))
		{
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER8, TIMER_INT_FLAG_CH1);//���жϹ���־
			
				tmp = timer_channel_capture_value_register_read(TIMER8, TIMER_CH_1);//��ȡ������ֵ 
				timer_counter_value_config(TIMER8, 0);//��ռ�����
			
				iTim1Value+=tmp;
				tmp=iTim1Value;
				iTim1Value=0;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_3))//�͵�ƽ
				{
						//timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//��һ��ʱ�̻��������� 
					  key_val1=1;
         }
				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
				{
						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0
					//	timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//�½���
						
				}
			
    }
		
			iDataTemp[iReCountEx++]=key_val1;//�ߵ͵�ƽ״̬
			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;

			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//��־
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';
			
		for(j=0;j<iReCountEx;j++)//��һ������
		{
			if(iReCount>(4096*4))break;//����						
			BUF_MOCAN[iReCount++]=iDataTemp[j];
		}
		
		
}


//time0 �����ж�
//		  
void TIMER0_UP_TIMER9_IRQnHandler(void)
{
//		/*u32 tmp=0,j=0*/;
//		u8 iDataTemp[10]={0};
//		u8 iReCountEx=0;
//		u8 key_val1=0;//��ƽ״̬
		
		//1*65535=65.535ms����һ������ж�
		if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_UP))	
		{
			timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
			iTim1Value+=0xFFFF;//TIM_GetCapture4(TIM5);//��������жϵ�ʱ��
			timer_counter_value_config(TIMER0, 0);//��ռ�����
			//TIM_SetCounter(TIM5,0);//��ռ�����
			return ;
		}		
	
//		
//		//TIME0 ֻ�е��߲���
//		//���ر仯
//		//�����ر仯ʱ �����ж�,��ʱ��ȡ������ ���ɻ����һ�ε�ƽ�仯��ʱ��
//    if(SET == timer_interrupt_flag_get(TIMER0, TIMER_INT_FLAG_CH2))
//		{
//        /* clear channel 0 interrupt bit */
//        timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_CH2);//���жϹ���־
//			
//				tmp = timer_channel_capture_value_register_read(TIMER0, TIMER_CH_2);//��ȡ������ֵ 
//				timer_counter_value_config(TIMER0, 0);//��ռ�����
//			
//				iTim1Value+=tmp;
//				tmp=iTim1Value;
//				iTim1Value=0;
//			
//			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//�͵�ƽ
//				{
//						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_RISING);//��һ��ʱ�̻��������� 
//					  key_val1=1;
//         }
//				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
//				{
//						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0
//						timer_channel_output_polarity_config(TIMER8,TIMER_CH_1,TIMER_IC_POLARITY_FALLING);//�½���
//						
//				}
//			
//    }
//	
//			iDataTemp[iReCountEx++]=key_val1;//�ߵ͵�ƽ״̬
//			
//			iDataTemp[iReCountEx++]=0x3b;
//			iDataTemp[iReCountEx++]=0x54;
//			iDataTemp[iReCountEx++]=0x69;

//			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
//			iDataTemp[iReCountEx++]=(tmp&0x0FF);
//			iDataTemp[iReCountEx++]=0x90;//��־
//			iDataTemp[iReCountEx++]='\r';
//			iDataTemp[iReCountEx++]='\n';
//			
//		for(j=0;j<iReCountEx;j++)//��һ������
//		{
//			if(iReCount>(4096*4))break;//����						
//			BUF_MOCAN[iReCount++]=iDataTemp[j];
//		}
		
		
}


/*!
    \brief      this function handles external lines 10 to 15 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
�ⲿ�ж� ��һ�����ڵĵ�ƽ�仯 �����ܼ�� ��ƽ��תʱ��

*/
void EXTI10_15_IRQHandler(void)
{
	
			u32 tmp=0,j=0;
		u8 iDataTemp[10]={0};
		u8 iReCountEx=0;
		u8 key_val1=0;//��ƽ״̬
    /* check the key tamper is pressed or not */
    if (RESET != exti_interrupt_flag_get(EXTI_10))
		{
        //gpio_bit_write(GPIOA, GPIO_PIN_10, (bit_status)(1-gpio_input_bit_get(GPIOE, GPIO_PIN_0)));
				tmp=GetTimer6Cnt();//��ʱ��6 1us ������;
			
			   if(RESET == gpio_input_bit_get(GPIOA,GPIO_PIN_10))//�͵�ƽ
				{
					  key_val1=1;
         }
				else//�ߵ�ƽ ��ʾ�¸�ʱ���� �͵�ƽ
				{
						key_val1=0;//�����Ǹߵ�ƽ ���� ��һ���� �͵�ƽ ���� ��0						
				}
			
        exti_interrupt_flag_clear(EXTI_10);
    }

			iDataTemp[iReCountEx++]=key_val1;//�ߵ͵�ƽ״̬			
			iDataTemp[iReCountEx++]=0x3b;
			iDataTemp[iReCountEx++]=0x54;
			iDataTemp[iReCountEx++]=0x69;
			iDataTemp[iReCountEx++]=(tmp&0x0FF0000)>>16;
			iDataTemp[iReCountEx++]=(tmp&0x0FF00)>>8;
			iDataTemp[iReCountEx++]=(tmp&0x0FF);
			iDataTemp[iReCountEx++]=0x90;//��־
			iDataTemp[iReCountEx++]='\r';
			iDataTemp[iReCountEx++]='\n';
			
		for(j=0;j<iReCountEx;j++)//��һ������
		{
			if(iReCount>(4096*4))break;//����						
			BUF_MOCAN[iReCount++]=iDataTemp[j];
		}
		
		GetTimer6Cnt();//��ʱ��6 1us ������  ���һ��
}

