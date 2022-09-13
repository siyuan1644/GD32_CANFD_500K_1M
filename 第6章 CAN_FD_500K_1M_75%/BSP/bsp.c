#include "bsp.h"
//#include "usart.h"
#include "timer.h"
#include	<string.h>
#include "cdc_acm_core.h"
#include "can.h"
//#include "vpwm.h"

__IO uint32_t   iTimeCmt = 0;                      //Send byte counter
__IO uint32_t   iTime5Cmt = 0;   //��ʱ��5������
__IO uint32_t   iTime6Cmt = 0;   //��ʱ��6������

__IO uint32_t iTim1Value=0;//��ʱ��1 ���벶�����ֵ
__IO uint8_t  iKwpIndexTime=0;//KWP ֡���ʱ��

//����������
	
	u8 BUF_MO[1024*4]={0};//4k    USB���տ����
 u8 BUF_MOCAN[1024*4]={0};//4k  ������ʱ�����
 
 u8 j1850_Hbit[500]={0};//J1850   �ߵ�ƽʱ��
 u8 j1850_Lbit[500]={0};//J1850   �͵�ƽʱ��
 __IO u16 iHBitSum=0;//�ߵ�ƽ����
__IO u16 iLBitSum=0;//�͵�ƽ ����
__IO u8 iJ1850EofFlag=0;//1850 ֡�������   1=֡����  
 
 u8 ReDataStr[1024*7]={0};//���ջ�����8K    USB ���Ϳ����
__IO u32 iReCount=0;//������֡��			

 
__IO uint8_t iFdCanFlag=0;//0 CAN2.0  ;80=FDCAN
__IO u8  iTimeFlag=0;
__IO u8 iIsoStatCom=0;//9141 ��ʼͨѶ��־
__IO u8 iKlineAdd=0;//K�� ��ַ��
__IO u16 iKlineBaud=0;//K�� ������
__IO u8 iKlineKey1=0,iKlineKey2=0;//��ַ��ģ��ʱ KEY1 KEY2
__IO u8 iPartValue=0;//��ǰЭ������  0=Stcan,1=Excan;2=KWP 14230 Fast;3=KWP 14230 Slow; 4 iso9141;  5 VWCAN  
 //6=1850 VPW  ;7=1850 PWM 
 //8=1850 ��ƽ��� Ĭ�� 0 
__IO u8 iKDataMode=0;// K�߲ɼ�����ģʽ 0 ʱ ����ģʽ  2ʱ �ɼ���ƽ  ��3 5�����ʲ���  =40 ģ��ECU �ȴ���ַ��

//�ٲ���
__IO u8 SJW=0;
__IO u8 BS1=0;
__IO u8 BS2=0;
__IO u8 Brp=0; 

//������
__IO u8 DataSJW=0;
__IO u8 DataBS1=0;
__IO u8 DataBS2=0;
__IO u8 DataBrp=0; 
//FD ���� 
__IO u8 FdMode=0;
__IO u8 FdBrs=0; 
__IO u8 Fd_Working_mode=0; //����ģʽ 0=����ģʽ  2=��Ĭģʽ(���ڼ���)

 
u8 Ledflag=0;

__IO u8 iEcuFlag=0;//�Ƿ���ģ��ECU  00�ɼ� ���� 80 ECU ģ��
__IO u16 iSendComValue=0;
__IO u16 iCanSendSum = 0;


__IO u8 iCan30flag=0;//���յ�30֡ ��� 00=��Ҫ�ȴ�30֡��80=�Ѿ����յ�30֡ 
__IO u8 iCan30Sum=0;//30֡�������͵Ķ�֡ ֡��   0��ʾ�����֡
__IO u16 iAdcValue=0;

__IO u16 KwpBaud=10416;//KWP ������
__IO u8 KwpAdd=0;//key1
__IO u8 KwpKey1=0;//key1
__IO u8 KwpKey2=0;//key2
__IO u8 KwpValue=0;//���յ���KWP ֵ
__IO u8 KwpReFlag=0;//KWP ���ձ��
__IO uint8_t iKwpLastSendValue=0;//���͵����һ���ֽ� K����ʱ�����յ��ֽڷ������һ���ֽ� ��֪��Ϊ��

__IO uint8_t iUsbRecFlag=0;//USB �������ݱ��  80���յ���λ�� ����

//�ⲿUSB ����
extern usb_core_driver cdc_acm;

 
 //ģ��ECU ʱ����30֡����λ�� 
void SendCan30Buf(void)
{
	if(iReCount==0) return ;
	 NewSendUsbDate(&cdc_acm,BUF_MOCAN,iReCount);
	 iReCount=0;
}
 
 //ADC ��ʼ��
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


 
 //��ʼ�� ��ͨIO ����
 //PB12  LED
// PB11  lin  
 void Led_Init(void)
{
		rcu_periph_clock_enable(RCU_GPIOB);
       /* configure LED2 GPIO port */ 
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_11|GPIO_PIN_12);
    /* reset LED2 GPIO pin */
    gpio_bit_reset(GPIOB,GPIO_PIN_12);

	//tja1027 оƬ���ƽŽ�Pb11 ,Pb11=1ʱʹ��оƬ
    gpio_bit_set(GPIOB,GPIO_PIN_11);	//Pb11=1����  0�ر�
	 // gpio_bit_reset(GPIOB,GPIO_PIN_11);	//Pb11=1����  0�ر�
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


//K �����ų�ʼ��  ������ͨID ����
//�������� ���ܲ�����ƽ
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



