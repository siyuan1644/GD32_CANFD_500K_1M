#include "bsp.h"
//#include "usart.h"
#include "timer.h"
#include	<string.h>
#include "cdc_acm_core.h"
#include "can.h"
//#include "vpwm.h"

__IO uint32_t   iTimeCmt = 0;                      //Send byte counter
__IO uint32_t   iTime5Cmt = 0;   //定时器5计数器
__IO uint32_t   iTime6Cmt = 0;   //定时器6计数器

__IO uint32_t iTim1Value=0;//定时器1 输入捕获溢出值
__IO uint8_t  iKwpIndexTime=0;//KWP 帧间隔时间

//缓冲区定义
	
	u8 BUF_MO[1024*4]={0};//4k    USB接收款冲区
 u8 BUF_MOCAN[1024*4]={0};//4k  数据临时款冲区
 
 u8 j1850_Hbit[500]={0};//J1850   高电平时间
 u8 j1850_Lbit[500]={0};//J1850   低电平时间
 __IO u16 iHBitSum=0;//高电平计数
__IO u16 iLBitSum=0;//低电平 计数
__IO u8 iJ1850EofFlag=0;//1850 帧结束标记   1=帧结束  
 
 u8 ReDataStr[1024*7]={0};//接收缓冲区8K    USB 发送款冲区
__IO u32 iReCount=0;//缓冲区帧数			

 
__IO uint8_t iFdCanFlag=0;//0 CAN2.0  ;80=FDCAN
__IO u8  iTimeFlag=0;
__IO u8 iIsoStatCom=0;//9141 开始通讯标志
__IO u8 iKlineAdd=0;//K线 地址码
__IO u16 iKlineBaud=0;//K线 波特率
__IO u8 iKlineKey1=0,iKlineKey2=0;//地址码模拟时 KEY1 KEY2
__IO u8 iPartValue=0;//当前协议类型  0=Stcan,1=Excan;2=KWP 14230 Fast;3=KWP 14230 Slow; 4 iso9141;  5 VWCAN  
 //6=1850 VPW  ;7=1850 PWM 
 //8=1850 电平检测 默认 0 
__IO u8 iKDataMode=0;// K线采集数据模式 0 时 正常模式  2时 采集电平  ，3 5波特率采样  =40 模拟ECU 等待地址码

//仲裁区
__IO u8 SJW=0;
__IO u8 BS1=0;
__IO u8 BS2=0;
__IO u8 Brp=0; 

//数据区
__IO u8 DataSJW=0;
__IO u8 DataBS1=0;
__IO u8 DataBS2=0;
__IO u8 DataBrp=0; 
//FD 参数 
__IO u8 FdMode=0;
__IO u8 FdBrs=0; 
__IO u8 Fd_Working_mode=0; //工作模式 0=正常模式  2=静默模式(用于监听)

 
u8 Ledflag=0;

__IO u8 iEcuFlag=0;//是否是模拟ECU  00采集 数据 80 ECU 模拟
__IO u16 iSendComValue=0;
__IO u16 iCanSendSum = 0;


__IO u8 iCan30flag=0;//接收到30帧 标记 00=需要等待30帧，80=已经接收到30帧 
__IO u8 iCan30Sum=0;//30帧后允许发送的多帧 帧数   0表示任意多帧
__IO u16 iAdcValue=0;

__IO u16 KwpBaud=10416;//KWP 波特率
__IO u8 KwpAdd=0;//key1
__IO u8 KwpKey1=0;//key1
__IO u8 KwpKey2=0;//key2
__IO u8 KwpValue=0;//接收到的KWP 值
__IO u8 KwpReFlag=0;//KWP 接收标记
__IO uint8_t iKwpLastSendValue=0;//发送的最后一个字节 K线有时候会接收到字节发的最后一个字节 不知道为何

__IO uint8_t iUsbRecFlag=0;//USB 接收数据标记  80接收到上位机 数据

//外部USB 定义
extern usb_core_driver cdc_acm;

 
 //模拟ECU 时发送30帧到上位机 
void SendCan30Buf(void)
{
	if(iReCount==0) return ;
	 NewSendUsbDate(&cdc_acm,BUF_MOCAN,iReCount);
	 iReCount=0;
}
 
 //ADC 初始化
 //PA7  adc in
void Adc_Init(void)
{
    /* enable GPIOA clock */
    rcu_periph_clock_enable(RCU_GPIOA);
	    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_AF);
    /* enable ADC clock */
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    //rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
	 gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_7);
	
	
	   /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1U);
    
    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_EXTTRIG_REGULAR_NONE); 
    /* ADC external trigger config */
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* enable ADC interface */
    adc_enable(ADC0);
    Delay_ms(1U);
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
}

/*!
    \brief      ADC channel sample
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t adc_channel_sample(uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_7POINT5);
    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while(!adc_flag_get(ADC0, ADC_FLAG_EOC));
    /* clear the end of conversion flag */
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(ADC0));
}


 
 //初始化 普通IO 引脚
 //PB12  LED
// PB11  lin  
 void Led_Init(void)
{
		rcu_periph_clock_enable(RCU_GPIOB);
       /* configure LED2 GPIO port */ 
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11|GPIO_PIN_12);
    /* reset LED2 GPIO pin */
    gpio_bit_reset(GPIOB,GPIO_PIN_12);

	//tja1027 芯片控制脚接Pb11 ,Pb11=1时使能芯片
    gpio_bit_set(GPIOB,GPIO_PIN_11);	//Pb11=1开启  0关闭
	 // gpio_bit_reset(GPIOB,GPIO_PIN_11);	//Pb11=1开启  0关闭
//	
//	
////		rcu_periph_clock_enable(RCU_GPIOA);
////		gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
}


/**
  * @brief  Toggles the specified GPIO pins.
  * @param  GPIOx: Where x can be (A..K) to select the GPIO peripheral.
  * @param  GPIO_Pin: Specifies the pins to be toggled.
  * @retval None
  */
void ToggleLed(void)
{	
	Ledflag++;
	if(Ledflag%2)
	{
		gpio_bit_set(GPIOB,GPIO_PIN_12);
	}
	else
	{
		gpio_bit_reset(GPIOB,GPIO_PIN_12);
	}
	
	
}


//K 线引脚初始化  当做普通ID 处理
//输入引脚 才能测量电平
void Kline_Init(void)
{
 		rcu_periph_clock_enable(RCU_GPIOA);
       /* configure LED2 GPIO port */ 
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,GPIO_PIN_10); 


//    rcu_periph_clock_enable(RCU_GPIOA);
//    rcu_periph_clock_enable(RCU_AF);
//    
//    /* configure button pin as input */
//    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
//    
//    /* enable and set key wakeup EXTI interrupt to the lowest priority */
//    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);

//    /* connect key wakeup EXTI line to key GPIO pin */
//    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_10);

//    /* configure key wakeup EXTI line */
//    exti_init(EXTI_10, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
//    exti_interrupt_flag_clear(EXTI_10);
	
}



