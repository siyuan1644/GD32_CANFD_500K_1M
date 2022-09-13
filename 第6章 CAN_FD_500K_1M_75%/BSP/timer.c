#include "timer.h"
#include "bsp.h"

//uint32_t Dmabuffer[512]={0};

///*��ʱ��1���ڼ�ʱ��*/
//void TIM1_config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//  /* Time base configuration */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//  TIM_DeInit(TIM1);
//  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;//��ֵ
//  TIM_TimeBaseStructure.TIM_Prescaler = 3599*2;//Ԥ��Ƶֵ
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;//ʱ�ӷָ�
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseStructure.TIM_RepetitionCounter = 4;
//  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//  /* Prescaler configuration *///0.1ms RCC
//  //TIM_PrescalerConfig(TIM1, 7199, TIM_PSCReloadMode_Immediate);
//  TIM_Cmd(TIM1, ENABLE);
//}


///*******************************************************************************
//* Function Name  : TIM3_config
//* Description    : ͨ�ö�ʱ��timer3��ʼ�� TIM_Period=������TIM_Prescaler=��Ƶϵ��
//* Input          : none
//* Output         : None
//* Return         : None  1ms ����
//*******************************************************************************/

//void TIM2_config(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_TimeBaseStructure.TIM_Period = 100-1; //  0~65535    1000*10us=1ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(840-1); // ��Ƶϵ��  10us 
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //ʱ�ӷָ�
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //
// 
//	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);  //ʱ���ж�
//	TIM_Cmd(TIM2, ENABLE);  //
//}



///*******************************************************************************
//* Function Name  : TIM3_config
//* Description    : ͨ�ö�ʱ��timer3��ʼ�� TIM_Period=������TIM_Prescaler=��Ƶϵ��
//* Input          : none
//* Output         : None
//* Return         : None


//*******************************************************************************/

//void TIM3_config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	
//	TIM_TimeBaseStructure.TIM_Period = 2-1; //  0~65535    1us*2=2us
//	TIM_TimeBaseStructure.TIM_Prescaler =(84-1); // ��Ƶϵ�� 84M/84=1000khz=1us
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //ʱ�ӷָ�
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //
// 
//	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
//	TIM_Cmd(TIM3, ENABLE);  //
//							 
//}

//��ʱ��5 16λ��1us ����
void TIM5_config(void)
{
	/* ----------------------------------------------------------------------------
    TIMER5 Configuration: 
    TIMER5CLK = SystemCoreClock/12000 = 10KHz, the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER5);

    timer_deinit(TIMER5);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
	
		timer_initpara.period            = 0xFFFE;//���ֵ
    timer_initpara.prescaler         = 120-1;//1us  
   // timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;//����Ƶ
    timer_init(TIMER5, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER5, TIMER_INT_UP);
    
    timer_enable(TIMER5);
		
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER5_IRQn, 1, 1);
		iTime5Cmt=0;//��ռ�����
}




//��ʱ��6 16λ��1us ����
//����������ж� ����ʱ�����
void TIM6_config(void)
{
	/* ----------------------------------------------------------------------------
    TIMER5 Configuration: 
    TIMER5CLK = SystemCoreClock/12000 = 10KHz, the period is 1s(10000/10000 = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER6);

    timer_deinit(TIMER6);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
	
		timer_initpara.period            = 0xFFFE;//���ֵ
    timer_initpara.prescaler         = 120-1;//1us  
   // timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;//����Ƶ
    timer_init(TIMER6, &timer_initpara);

    /* enable the TIMER interrupt */
    timer_interrupt_flag_clear(TIMER6, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER6, TIMER_INT_UP);
    
    timer_enable(TIMER6);
//		
		nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER6_IRQn, 1, 1);
		iTime6Cmt=0;//��ռ�����
}


/*********************************
TIME5 ��ʱ us ����


**********************************/
void Time5Delayus(uint32_t us)
{
		uint32_t iCmt=0;
		uint32_t iTemp=0; 
		uint32_t iSum=us/10000;//�ֳɺܶ��
	  for(uint32_t i=0;i<iSum;i++)
		{
			iTime5Cmt=0;
			timer_counter_value_config(TIMER5,0);//��������0
			while(1)
			{
				iCmt=timer_counter_read(TIMER5);//��ȡ������ ֵ		
				if((iCmt+iTime5Cmt)>=10000) break;		
				
			}
		}
		iTemp=us%10000;
		if(iTemp>0)
		{
			iTime5Cmt=0;
			timer_counter_value_config(TIMER5,0);//��������0
			while(1)
			{
				iCmt=timer_counter_read(TIMER5);//��ȡ������ ֵ		
				if((iCmt+iTime5Cmt)>=iTemp) break;		
				
			}
		}			
		
			//����ж�ʱ���ܸպñ����� ���Ի᲻׼ȷ
//		iTime5Cmt=0;
//		timer_counter_value_config(TIMER5,0);//��������0
//		while(1)
//		{
//			iCmt=timer_counter_read(TIMER5);//��ȡ������ ֵ		
//			if((iCmt+iTime5Cmt)>=us) break;		
//			
//		}
		
}
/*****************************
1us ��ʱ����


*****************************/
void Delay_us(uint32_t us)
{
	Time5Delayus(us);
}

void Delay_ms(uint32_t ms)
{
	Time5Delayus(ms*1000);
}

//��ȡʱ��ֵ
uint32_t GetTimer6Cnt(void)
{
	uint32_t ti=0;
	ti=timer_counter_read(TIMER6);//��ȡ������ ֵ	
	ti+=iTime6Cmt;
	timer_counter_value_config(TIMER6,0);//��������0
	iTime6Cmt=0;//���ȫ�ּ�����
	return ti;
}

//��ȡʱ��ֵ
//���������  ������ֵ ���ڳ�ʱ����
uint32_t GetTimer6CntEx(void)
{
	uint32_t ti=0;
	ti=timer_counter_read(TIMER6);//��ȡ������ ֵ	
	ti+=iTime6Cmt;
//	timer_counter_value_config(TIMER6,0);//��������0
//	iTime6Cmt=0;//���ȫ�ּ�����
	return ti;
}
//vpwm ����

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA0/PA1/PA2(TIMER1 CH0/CH1/CH2) as alternate function*/
//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
//    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void Time1PwmInit(void)
{
    /* -----------------------------------------------------------------------
    TIMER1 configuration: generate 3 PWM signals with 3 different duty cycles:
    TIMER1CLK = SystemCoreClock / 120 = 1MHz, the PWM frequency is 62.5Hz.

    TIMER1 channel0 duty cycle = (4000/ 16000)* 100  = 25%
    TIMER1 channel1 duty cycle = (8000/ 16000)* 100  = 50%
    TIMER1 channel2 duty cycle = (12000/ 16000)* 100 = 75%
    ----------------------------------------------------------------------- */
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

	
		gpio_config();//��ʼ������
	
	
		//��ʼ����ʱ��5 TIM5	 
//	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; //�趨�������Զ���װֵ 
//	TIM_TimeBaseStructure.TIM_Prescaler =84-1; 	//Ԥ��Ƶ��   
	
    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
		timer_initpara.period            = 1000-1;//����
    timer_initpara.prescaler         = 120-1;//1us
		
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1, &timer_initpara);

    /* initialize TIMER channel output parameter struct */
    timer_channel_output_struct_para_init(&timer_ocinitpara);
    /* CH0, CH1 and CH2 configuration in PWM mode */
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocinitpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

//    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);
//    timer_channel_output_config(TIMER1, TIMER_CH_1, &timer_ocinitpara);
    timer_channel_output_config(TIMER1, TIMER_CH_2, &timer_ocinitpara);

//    /* CH0 configuration in PWM mode0, duty cycle 25% */
//    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 4000);
//    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
//    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

//    /* CH1 configuration in PWM mode0, duty cycle 50% */
//    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 8000);
//    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
//    timer_channel_output_shadow_config(TIMER1, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* CH2 configuration in PWM mode0, duty cycle 75% */
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 800);//ռ�ձ�  ���ߵ�ƽʱ��
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* auto-reload preload enable */
    timer_enable(TIMER1);
}



/*****************************************************
TIME1 CH3���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���
input capture both edge(only for TIMER8~TIMER13)
PA3
*****************************************************/
void TIM1_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA3(TIMER1 CH3) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
	
		 /* configure LED2 GPIO port */ 
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2);
    /* reset LED2 GPIO pin */
	//TX ��������
    gpio_bit_set(GPIOA,GPIO_PIN_2);	//PA2=1
	
	
	//nvic
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
	
	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;//1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER2  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;//�½���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;//�˲��� 
    timer_input_capture_config(TIMER1,TIMER_CH_3,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH3|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_CH3|TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER1);
	
}


/*****************************************************
TIME1 CH0 PA0 PWM ���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���

*****************************************************/
void TIM1_CH0_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA3(TIMER1 CH3) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	
	
	//nvic
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
	
	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;// 1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER2  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;////OBD 2��ʼ��ƽ�ǵ͵�ƽ ����Ҫ���ó� �����ش���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 10;//�˲��� 
    timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_CH0|TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER1);
	
}



/*****************************************************
DMA TIME1 CH0 PA0 PWM ���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���

*****************************************************/
void DMA_TIM1_CH0_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA3(TIMER1 CH3) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
	
	
	//nvic
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
	
	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;// 1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER2  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;////OBD 2��ʼ��ƽ�ǵ͵�ƽ ����Ҫ���ó� �����ش���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 10;//�˲��� 
    timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    //timer_interrupt_enable(TIMER1,TIMER_INT_CH0|TIMER_INT_UP);
		timer_interrupt_enable(TIMER1,TIMER_INT_UP);//ֻ�������ж�
		
		/* TIMER0 update DMA request enable */
   // timer_dma_enable(TIMER1, TIMER_DMA_UPD);
		
		timer_dma_enable(TIMER1, TIMER_DMA_CH0D);
    
		/* TIMER2 counter enable */
    timer_enable(TIMER1);
	
}
/*****************************************************
TIME1 CH1 PA1 VPW ���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���

*****************************************************/
void TIM1_CH1_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA3(TIMER1 CH3) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	
	
	//nvic
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
	
	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;//1us 
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    /* TIMER2  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;//OBD 2��ʼ��ƽ�ǵ͵�ƽ ����Ҫ���ó� �����ش���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x3;//�˲��� 
    timer_input_capture_config(TIMER1,TIMER_CH_1,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH1|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER1,TIMER_INT_CH1|TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER1);
	
}

/*****************************************************
TIME8 CH1 PA3   ���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���
input capture both edge(only for TIMER8~TIMER13)
PA3
*****************************************************/
void TIM8_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /*configure PA3(TIMER1 CH3) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
	
	//nvic
	  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER0_BRK_TIMER8_IRQn, 1, 1);//��TIME0 
	
	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER8);

    timer_deinit(TIMER8);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;//1us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER8, &timer_initpara);

    /* TIMER8  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_BOTH_EDGE;//˫�߲���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER8,TIMER_CH_1,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER8);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER8,TIMER_INT_FLAG_CH1|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER8,TIMER_INT_CH1|TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER8);
	
}


/*****************************************************
TIME0 CH2 PA10   ���벶��
GD32 ֻ�ж�ʱ�� TIMER8~TIMER13��˫�߲�����
������ʱ��ֻ�е��߲���
input capture both edge(only for TIMER8~TIMER13)

//PA9  TX TIME0 CH1
//PA10 RX TIME0 CH2
�޷�����ͨ�� ��ʱ��֪������
*****************************************************/
void TIM0_Cap_Init(void)
{	 
	  rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
		//rcu_periph_clock_enable(RCU_TIMER0);
	//	gpio_pin_remap_config(GPIO_TIMER0_FULL_REMAP,ENABLE);//
		
    /*configure PA3(TIMER1 CH3) as alternate function*/
   gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
	
	 /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
   // gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
	
	
	//nvic
	 // nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER0_UP_TIMER9_IRQn, 1, 1);//��TIME0 
	  nvic_irq_enable(TIMER0_Channel_IRQn, 1, 0);

	
	 /* TIMER2 configuration: input capture mode -------------------
    the external signal is connected to TIMER1 CH3 pin(PA3)
    the rising edge is used as active edge
    the TIMER2 CH0CV is used to compute the frequency value
    ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER0);

    timer_deinit(TIMER0);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
		
		timer_initpara.period            = 65535;//������
    timer_initpara.prescaler         = 120-1;//1us
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
		timer_initpara.repetitioncounter     = 0;
    timer_init(TIMER0, &timer_initpara);

    /* TIMER8  configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;//�½���
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_capture_config(TIMER0,TIMER_CH_2,&timer_icinitpara);
																
    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER0);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER0,TIMER_INT_FLAG_CH2|TIMER_INT_UP);
    /* channel 0 interrupt enable */
    timer_interrupt_enable(TIMER0,TIMER_INT_CH2|TIMER_INT_UP);

    /* TIMER2 counter enable */
    timer_enable(TIMER0);
	
}






/**
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void dma_config(void)
{
	
//TIMER1����ַ��0x40000000 
//TIMER2����ַ��0x40000400 
//TIMER3����ַ��0x40000800 
//TIMER4����ַ��0x40000C00 
//TIMER0����ַ��0x40012C00
//#define TIMER0_CH0CV                    ((uint32_t)0x040012C34)	//����ַ+ƫ�Ƶ�ַ
//#define TIMER1_CH0CV                    ((uint32_t)0x040000034)	//����ַ+ƫ�Ƶ�ַ PWM time1 CH0 ��ַ
	
//     dma_parameter_struct dma_init_struct;

//    /* enable DMA clock */
//    rcu_periph_clock_enable(RCU_DMA0);

//    /* initialize DMA channel5 */
//    dma_deinit(DMA0, DMA_CH4);
//    /* DMA channel5 initialize */
//    dma_init_struct.periph_addr  = (uint32_t)TIMER1_CH0CV;
//    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;//�����ڴ�����Ƿ�ʹ��
//		dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
//    dma_init_struct.memory_addr  = (uint32_t)Dmabuffer;//�ڴ��ַ
//    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;//�ڴ����
//    
//    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
//    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
//    dma_init_struct.number       = sizeof(Dmabuffer);
//    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
//    dma_init(DMA0, DMA_CH4, &dma_init_struct);
//    
//    dma_circulation_enable(DMA0, DMA_CH4);

//    /* enable DMA channel5 */
//    dma_channel_enable(DMA0, DMA_CH4);
}











