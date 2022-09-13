#include "can.h"
#include "BSP.h"
#include	<string.h>
#include "timer.h"


//GD32 60M
//
const uint8_t CANBAUD[10][4]={
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_1TQ,15}, //0 1M  
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_3TQ,CAN_BT_BS2_1TQ,15},  //1 800k      80%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,15},  //2 500k      87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,30}, //3 250k			 87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_13TQ,CAN_BT_BS2_2TQ,30} //4 125k			 87.5% 
 
 
 
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,20},//5 100k 
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,83}, //6 62.5k  42M/((1+5+2)*83)
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,60}, //7 50k
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,210},//8 33.3k  42M/((1+2+3)*210)
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,84},//9 25k
};



//������
//60M ʱ��Ƶ��
//BSJ,��Ƶ��BS1,BS2
//���ֵ 0%
const u16 CANBAUD_data[20][4]={
	
	
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,15},   //0 500k      87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_6TQ,CAN_BT_BS2_1TQ,30},  //1 250k       87.5%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_13TQ,CAN_BT_BS2_2TQ,30}, //2 125k        87.5%
// {CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_1TQ,15},  //3 1M        75%
  {CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_2TQ,12},  //3 1M        83%
 //{CAN_BT_SJW_1TQ,CAN_BT_BS1_10TQ,CAN_BT_BS2_4TQ,2},  //4 2M        73.3%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_12TQ,CAN_BT_BS2_2TQ,2},  //4 2M        73.3%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_2TQ,CAN_BT_BS2_1TQ,5},   //5 3M        75.5%   
 {CAN_BT_SJW_1TQ, CAN_BT_BS1_11TQ, CAN_BT_BS2_3TQ, 1},  //6 4M        80%   OK
 //{CAN_BT_SJW_1TQ,CAN_BT_BS1_8TQ,CAN_BT_BS2_3TQ,1},   //7 5M        75%
 {CAN_BT_SJW_1TQ,CAN_BT_BS1_4TQ,CAN_BT_BS2_1TQ,2},   //7 5M        83.3%   OK
 //{CAN_BT_SJW_1TQ, CAN_BT_BS1_10TQ, CAN_BT_BS2_1TQ, 1},   //7 5M        91%   ����������
 //{CAN_BT_SJW_1TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 1},   //7 5M 
 {1,1,6,3},     //8 8M        70%  ��֧��
 
};

//		CAN_InitSt.resync_jump_width=CAN_BT_SJW_1TQ;
//		CAN_InitSt.time_segment_1=CAN_BT_BS1_6TQ;
//		CAN_InitSt.time_segment_2=CAN_BT_BS2_1TQ;

//����CAN FD ����
//�ٲ���
extern __IO u8 SJW;
extern __IO u8 BS1;
extern __IO u8 BS2;
extern __IO u8 Brp; 

//������
extern __IO u8 DataSJW;
extern __IO u8 DataBS1;
extern __IO u8 DataBS2;
extern __IO u8 DataBrp; 
//FD ���� 
extern __IO u8 FdMode;
extern __IO u8 FdBrs; 


#define DEV_CAN0_ID          0xaabb
#define DEV_CAN0_MASK        0x0000
#define DEV_CAN1_ID          0xccdd
#define DEV_CAN1_MASK        0x0000
/* config CAN baud rate to 500K Hz (range from 1Hz to 1MHz)*/
#define DEV_CAN_BAUD_RATE    500000

//GD32  CAN =60M

/*!
    \brief      configure GPIO
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_gpio_config(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
   
		gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP,ENABLE);//
	
		gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);//RX
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);//TX
	
//    /* configure CAN0 GPIO */
//    gpio_init(GPIOD,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
//    gpio_init(GPIOD,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_1);
//    
//    /* configure CAN1 GPIO */
//    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);//RX
//    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);//TX
    
   // gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP,ENABLE);
   // gpio_pin_remap_config(GPIO_CAN1_REMAP,ENABLE);
}

/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_config(void)
{
    can_parameter_struct can_parameter;
     
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);
  //  can_deinit(CAN1);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;  
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
   // can_init(CAN1, &can_parameter);
    
    /* config CAN0 baud rate */
    can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);
    /* config CAN1 baud rate */
   // can_frequency_set(CAN1, DEV_CAN_BAUD_RATE);
    
    /* initialize filter */ 
    can1_filter_start_bank(14);
    can_filter_mask_mode_init(DEV_CAN0_ID, DEV_CAN0_MASK, CAN_STANDARD_FIFO0, 0);
    //can_filter_mask_mode_init(DEV_CAN1_ID, DEV_CAN1_MASK, CAN_EXTENDED_FIFO0, 15);
    
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* configure CAN1 NVIC */
  //  nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
  //  can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}



/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void can_configEx(uint8_t speed)
{
    can_parameter_struct CAN_InitSt;
	
     //GPIO
		can_gpio_config();
	
    can_struct_para_init(CAN_INIT_STRUCT, &CAN_InitSt);
    /* initialize CAN register */
    can_deinit(CAN0);
    
    /* initialize CAN parameters */
    CAN_InitSt.time_triggered = DISABLE;
    CAN_InitSt.auto_bus_off_recovery = DISABLE;
    CAN_InitSt.auto_wake_up = DISABLE;
	
//		0��ʹ���Զ��ط�  ENABLE
//		1�������Զ��ط�
    CAN_InitSt.auto_retrans = ENABLE;//�����Զ����� �Ƿ��� 
	
    CAN_InitSt.rec_fifo_overwrite = DISABLE;
    CAN_InitSt.trans_fifo_order = DISABLE;
    //CAN_InitSt.working_mode = Fd_Working_mode;  //����ģʽ  �ĳ�����λ������
	 CAN_InitSt.working_mode = CAN_NORMAL_MODE;  //��Ĭģʽ ���ڼ�����������  ����������λ �������ж���ڵ�ʱ ����ѡ�����ģʽ
	
	//SPEED
		CAN_InitSt.resync_jump_width=CANBAUD[speed][0];
		CAN_InitSt.time_segment_1=CANBAUD[speed][1];
		CAN_InitSt.time_segment_2=CANBAUD[speed][2];
		CAN_InitSt.prescaler=CANBAUD[speed][3];//BAUD=60m/((1+6+1)*15)  87.5%
		
		/* initialize CAN */
    can_init(CAN0, &CAN_InitSt);

  
    
		//fiter 
    /* initialize filter */ 
   // can1_filter_start_bank(14);
  //  can_filter_mask_mode_init(0x7E8, 0x7E8, CAN_STANDARD_FIFO0, 0);

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}




/*!
    \brief      initialize CAN  FD function
    \speed1 �ٲ���������
		\speed2 ������������
    \param[out] none
    \retval     none
*/
void CanFD_config(uint8_t speed1,uint8_t speed2)
{
    can_parameter_struct CAN_InitSt;
		can_fdframe_struct can_fd_parameter; //CAN FD ����
		can_fd_tdc_struct can_fd_tdc_parameter;//	
	
     //GPIO
		can_gpio_config();
	
    can_struct_para_init(CAN_INIT_STRUCT, &CAN_InitSt);
    /* initialize CAN register */
    can_deinit(CAN0);
    
	
	
    /* initialize CAN parameters */
    CAN_InitSt.time_triggered = DISABLE;
    CAN_InitSt.auto_bus_off_recovery = DISABLE;
    CAN_InitSt.auto_wake_up = DISABLE;
	
//		0��ʹ���Զ��ط�
//		1�������Զ��ط�
    CAN_InitSt.auto_retrans = ENABLE;//�����Զ����� �Ƿ��� 
    CAN_InitSt.rec_fifo_overwrite = DISABLE;
    CAN_InitSt.trans_fifo_order = DISABLE;
    CAN_InitSt.working_mode = CAN_NORMAL_MODE;  
  
	//speed1 �ٲ���������
		CAN_InitSt.resync_jump_width=CANBAUD[speed1][0];
		CAN_InitSt.time_segment_1=CANBAUD[speed1][1];
		CAN_InitSt.time_segment_2=CANBAUD[speed1][2];
		CAN_InitSt.prescaler=CANBAUD[speed1][3];//BAUD=60m/((1+6+1)*15)  87.5%
		
		/* initialize CAN */
    can_init(CAN0, &CAN_InitSt);
		//can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);
  
    //������ ��ʼ��
		can_struct_para_init(CAN_FD_FRAME_STRUCT, &can_fd_parameter);
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
		
    can_fd_tdc_parameter.tdc_filter = 0x04; 
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = CAN_FDMOD_ISO;
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;
    
    //����������������
		can_fd_parameter.data_resync_jump_width=CANBAUD_data[speed2][0];
		can_fd_parameter.data_time_segment_1=CANBAUD_data[speed2][1];
		can_fd_parameter.data_time_segment_2=CANBAUD_data[speed2][2];
		can_fd_parameter.data_prescaler=CANBAUD_data[speed2][3];//BAUD=60m/((1+6+1)*15)  
		can_fd_init(CAN0, &can_fd_parameter);
		
  //can_fd_frequency_set(CAN0, 5000000);//1M  �Զ������ʺ���
		
		

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}


/*!
    \brief      initialize CAN function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void canFD_configEx(uint8_t speed1,uint8_t speed2)
{
    can_parameter_struct can_parameter;
    can_fdframe_struct can_fd_parameter; 
    can_fd_tdc_struct can_fd_tdc_parameter;
    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    /* initialize CAN register */
    can_deinit(CAN0);
    //can_deinit(CAN1);
    
    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = ENABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.working_mode = CAN_NORMAL_MODE;  
	
		can_parameter.resync_jump_width=CANBAUD[speed1][0];
		can_parameter.time_segment_1=CANBAUD[speed1][1];
		can_parameter.time_segment_2=CANBAUD[speed1][2];
		can_parameter.prescaler=CANBAUD[speed1][3];//BAUD=60m/((1+6+1)*15)  87.5%
	
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    //can_init(CAN1, &can_parameter);
   
    /* config CAN0 baud rate */
   // can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);
    /* config CAN1 baud rate */
    //can_frequency_set(CAN1, DEV_CAN_BAUD_RATE);
    
    can_struct_para_init(CAN_FD_FRAME_STRUCT, &can_fd_parameter);
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
    can_fd_tdc_parameter.tdc_filter = 0x04; 
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = CAN_FDMOD_ISO;
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;
    can_fd_init(CAN0, &can_fd_parameter);
    //can_fd_init(CAN1, &can_fd_parameter);
    
    can_fd_frequency_set(CAN0, 4000000);
    //can_fd_frequency_set(CAN1, 2000000);
    
//    /* initialize filter */ 
//    can1_filter_start_bank(14);
//    can_filter_mask_mode_init(DEV_CAN0_ID, DEV_CAN0_MASK, CAN_EXTENDED_FIFO0, 0);
//    can_filter_mask_mode_init(DEV_CAN1_ID, DEV_CAN1_MASK, CAN_EXTENDED_FIFO0, 15);
//    
    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* configure CAN1 NVIC */
    //nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    //can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}


/*
*********************************************************************************************************
*                                          CAN1_Config16BitFilter()
*
* ����   �� ����CAN�˲�����������16λ��׼֡ID
*
* ����   �� id1 ��Ҫ����һ��16λ��׼֡ID
*
*           id2 ��Ҫ������һ��16λ��׼֡ID
*
* ����ֵ �� ��
*
* ע��   �� ��
*********************************************************************************************************
*/
void CAN1_Config16BitFilter(u16 id1, u16 id2)
{
    can_filter_parameter_struct  CAN_FilterInitStructure;
		can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);

		CAN_FilterInitStructure.filter_number = 0;
    CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
    CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_16BIT;
	
    CAN_FilterInitStructure.filter_enable = ENABLE;
		 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
    /* configure SFID[10:0] */
    CAN_FilterInitStructure.filter_list_high = (uint16_t)id1 ;
    CAN_FilterInitStructure.filter_list_low = (uint16_t)id2;
    /* configure SFID[10:0] mask */
    CAN_FilterInitStructure.filter_mask_high = (uint16_t)id1;
    /* both data and remote frames can be received */
    CAN_FilterInitStructure.filter_mask_low = (uint16_t)id2 ;	
		can_filter_init(&CAN_FilterInitStructure);
}


//������ID
void CAN_setAllfit(void)
{
		can_filter_parameter_struct  CAN_FilterInitStructure;
		can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);

		CAN_FilterInitStructure.filter_number = 0;
    CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_MASK;
    CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_32BIT;
	
    CAN_FilterInitStructure.filter_enable = ENABLE;
		 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
    /* configure SFID[10:0] */
    CAN_FilterInitStructure.filter_list_high = (uint16_t)0 ;
    CAN_FilterInitStructure.filter_list_low = (uint16_t)0;
    /* configure SFID[10:0] mask */
    CAN_FilterInitStructure.filter_mask_high = (uint16_t)0;
    /* both data and remote frames can be received */
    CAN_FilterInitStructure.filter_mask_low = (uint16_t)0 ;	
		can_filter_init(&CAN_FilterInitStructure);
}



// �б�ģʽ �� 0~13 �� ÿ����� 4��STCAN ID ���4*28=112 ID
// FIFO 0  0~27
// FIFO 1  0~27 
//FIFO ���� 0~13 ��������
void CAN1_Config16BitFilterList(u16 *iBuffer,u8 iSumFiter)
{
    can_filter_parameter_struct  CAN_FilterInitStructure;
		u16 id1=0,id2=0,id3=0,id4=0;	
		u8 i=0,iCmt=0;
		u8 iSum=0;
		
		iSum=iSumFiter/4;
		if(iSumFiter%4)
		{
			iSum+=1;
		}
		if(iSum>27) iSum=27;	//fifo 1
	
		for(i=0;i<iSum;i++)//0~13  sum=14*2 
		{			
//			if(i>13) iFiterCount=i-14;//fifo 1
//			else iFiterCount=i;

			id1=iBuffer[iCmt++];
			id2=iBuffer[iCmt++];
			id3=iBuffer[iCmt++];
			id4=iBuffer[iCmt++];
			if(id1==0) id1=0xffff;
			if(id2==0) id2=0xffff;
			if(id3==0) id3=0xffff;
			if(id4==0) id4=0xffff;		
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_16BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint16_t)id1 ;
			CAN_FilterInitStructure.filter_list_low = (uint16_t)id2;
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint16_t)id3;
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint16_t)id4 ;	
			can_filter_init(&CAN_FilterInitStructure);

		}

 
    
}


// ����ģʽ �� 0~27 �� ÿ����� 2��STCAN ID ���2*28=56�� ID
// FIFO 0  0~27
// FIFO 1  0~27 
//FIFO ���� 0~27 ��������
void InitStcanFiterMask(u16 *iBuffer,u8 iSumFiter)
{
    can_filter_parameter_struct  CAN_FilterInitStructure;
		u16 id1=0,id2=0,id1Mask=0,id2Mask=0;	
		u8 i=0,iCmt=0;
		u8 iSum=0;
		
		iSum=iSumFiter/4;
		if(iSumFiter%4)
		{
			iSum+=1;
		}
		if(iSum>27 )iSum=27;	//fifo 1
	
		for(i=0;i<iSum;i++)//0~13  sum=14*2 
		{			

			
			id1=iBuffer[iCmt++];//����λ
			id1Mask=iBuffer[iCmt++];//����
			
			id2=iBuffer[iCmt++];//����λ
			id2Mask=iBuffer[iCmt++];//����			
			if(id1==0) id1=0xffff;
			if(id1Mask==0) id1Mask=0xffff;
			if(id2==0) id2=0xffff;
			if(id2Mask==0) id2Mask=0xffff;	
	
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_MASK;//���뷽ʽ
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_16BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint16_t)id1 ;//ID 1
			CAN_FilterInitStructure.filter_list_low = (uint16_t)id2;  //ID 2
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint16_t)id1Mask;//���� 1
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint16_t)id2Mask ;	//���� 2
			can_filter_init(&CAN_FilterInitStructure);

		}

 
    
}

//�б�ģʽ��  ��0~13 ��14�� �ܹ��� 28����չ֡ID
void CAN1_Config32BitFilterExList(u32 *iBuffer,u8 iSumFiter)
{
    u32 id1=0,id2=0;
		u8 i=0,iCmt=0,iSum=0;
    can_filter_parameter_struct  CAN_FilterInitStructure;
	//  id1=id1>>3;
		
		iSum=iSumFiter/2;
		if(iSumFiter%2)
		{
			iSum+=1;
		}
	
	
	
		for(i=0;i<iSum;i++)
		{
			id1=iBuffer[iCmt++];
			id2=iBuffer[iCmt++];
			
			id1=id1>>3;
			id2=id2>>3;
			
			if(id1==0) id1=0xffffffff;
			if(id2==0) id2=0xffffffff;
			
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_LIST;
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_32BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint32_t)(id1>>13);
			CAN_FilterInitStructure.filter_list_low = (uint32_t)((id1<<3)|4);
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint32_t)(id2>>13);
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint32_t)((id2<<3)|4);
			can_filter_init(&CAN_FilterInitStructure);
			
		}
	

  
	

}


//���������չ֡���Թ��� 1*28 ��������
void InitExcanFiterMask(u32 *iBuffer,u8 iSumFiter)
{
    u32 id1=0,id2=0;
		u8 i=0,iCmt=0,iSum=0;
    can_filter_parameter_struct  CAN_FilterInitStructure;
	//  id1=id1>>3;
		
		iSum=iSumFiter/2;
		if(iSumFiter%2)
		{
			iSum+=1;
		}
	
		if(iSum>27) iSum=27;
	
		for(i=0;i<iSum;i++)
		{
			
				
			id1=iBuffer[iCmt++];
			id2=iBuffer[iCmt++];
			
			id1=id1>>3;
			id2=id2>>3;
			
			if(id1==0) id1=0xffffffff;
			if(id2==0) id2=0xffffffff;
			
			can_struct_para_init(CAN_FILTER_STRUCT, &CAN_FilterInitStructure);
			CAN_FilterInitStructure.filter_number = i;
			CAN_FilterInitStructure.filter_mode = CAN_FILTERMODE_MASK;
			CAN_FilterInitStructure.filter_bits = CAN_FILTERBITS_32BIT;
		
			CAN_FilterInitStructure.filter_enable = ENABLE;
			 CAN_FilterInitStructure.filter_fifo_number = CAN_FIFO0;
			/* configure SFID[10:0] */
			CAN_FilterInitStructure.filter_list_high = (uint32_t)(id1>>13);
			CAN_FilterInitStructure.filter_list_low = (uint32_t)((id1<<3)|4);//ID
			/* configure SFID[10:0] mask */
			CAN_FilterInitStructure.filter_mask_high = (uint32_t)(id2>>13);//����
			/* both data and remote frames can be received */
			CAN_FilterInitStructure.filter_mask_low = (uint32_t)((id2<<3)|4);
			can_filter_init(&CAN_FilterInitStructure);
			
		}
	

  
	

}

//�����־λ
void ClearFdFlag(void)
{

//	  CAN_FLAG_TMLS2    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 31U),          /*!< transmit mailbox 2 last sending in Tx FIFO */ 
//    CAN_FLAG_TMLS1    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 30U),          /*!< transmit mailbox 1 last sending in Tx FIFO */ 
//    CAN_FLAG_TMLS0    = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 29U),          /*!< transmit mailbox 0 last sending in Tx FIFO */ 
//    CAN_FLAG_TME2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 28U),          /*!< transmit mailbox 2 empty */
//    CAN_FLAG_TME1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 27U),          /*!< transmit mailbox 1 empty */
//    CAN_FLAG_TME0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 26U),          /*!< transmit mailbox 0 empty */
//    CAN_FLAG_MTE2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 19U),          /*!< mailbox 2 transmit error */
//    CAN_FLAG_MTE1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 11U),          /*!< mailbox 1 transmit error */
//    CAN_FLAG_MTE0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 3U),           /*!< mailbox 0 transmit error */
//    CAN_FLAG_MTF2     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 16U),          /*!< mailbox 2 transmit finished */
//    CAN_FLAG_MTF1     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 8U),           /*!< mailbox 1 transmit finished */
//    CAN_FLAG_MTF0     = CAN_REGIDX_BIT(TSTAT_REG_OFFSET, 0U),           /*!< mailbox 0 transmit finished */
	
//	    CAN_FLAG_BOERR    = CAN_REGIDX_BIT(ERR_REG_OFFSET, 2U),             /*!< bus-off error */ 
//    CAN_FLAG_PERR     = CAN_REGIDX_BIT(ERR_REG_OFFSET, 1U),             /*!< passive error */ 
//    CAN_FLAG_WERR     = CAN_REGIDX_BIT(ERR_REG_OFFSET, 0U),             /*!< warning error */ 
	
	if(can_flag_get(CAN1,CAN_FLAG_TMLS2)) can_flag_clear(CAN1,CAN_FLAG_TMLS2);
	if(can_flag_get(CAN1,CAN_FLAG_TMLS1)) can_flag_clear(CAN1,CAN_FLAG_TMLS1);
	if(can_flag_get(CAN1,CAN_FLAG_TMLS0)) can_flag_clear(CAN1,CAN_FLAG_TMLS0);
	if(can_flag_get(CAN1,CAN_FLAG_TME2)) can_flag_clear(CAN1,CAN_FLAG_TME2);
	if(can_flag_get(CAN1,CAN_FLAG_TME1)) can_flag_clear(CAN1,CAN_FLAG_TME1);
	if(can_flag_get(CAN1,CAN_FLAG_TME0)) can_flag_clear(CAN1,CAN_FLAG_TME0);
	if(can_flag_get(CAN1,CAN_FLAG_MTE2)) can_flag_clear(CAN1,CAN_FLAG_MTE2);
	if(can_flag_get(CAN1,CAN_FLAG_MTE1)) can_flag_clear(CAN1,CAN_FLAG_MTE1);
	if(can_flag_get(CAN1,CAN_FLAG_MTE0)) can_flag_clear(CAN1,CAN_FLAG_MTE0);
	
	if(can_flag_get(CAN1,CAN_FLAG_MTF2)) can_flag_clear(CAN1,CAN_FLAG_MTF2);
	if(can_flag_get(CAN1,CAN_FLAG_MTF1)) can_flag_clear(CAN1,CAN_FLAG_MTF1);
	if(can_flag_get(CAN1,CAN_FLAG_MTF0)) can_flag_clear(CAN1,CAN_FLAG_MTF0);
	
	if(can_flag_get(CAN1,CAN_FLAG_BOERR)) can_flag_clear(CAN1,CAN_FLAG_BOERR);
	if(can_flag_get(CAN1,CAN_FLAG_PERR)) can_flag_clear(CAN1,CAN_FLAG_PERR);
	if(can_flag_get(CAN1,CAN_FLAG_WERR)) can_flag_clear(CAN1,CAN_FLAG_WERR);
		
}



//CAN 2.0 TEST
void StCanInitTest(void)
{
	can_gpio_config();
	//can_config();
	can_configEx(can_500k);
	CAN1_Config16BitFilter(0xFC00,0xFD00);
	//CAN_setAllfit();
}

//CAN FD TEST
void FdCanInitTest(void)
{
	can_gpio_config();
	//can_config();
	//canFD_configEx(can_500k,Data_2M);
	
	CanFD_config(can_500k,Data_5M);//500k  2M

	//CAN1_Config16BitFilter(0xFC00,0xFD00);
	CAN_setAllfit();
}

//����������
u8 CanTest(u8 iPoi)
{
	u8 iErrorCmt=0;
		can_trasnmit_message_struct g_transmit_message;
	  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = 0x07E8;
    g_transmit_message.tx_efid = 0x00;
    g_transmit_message.tx_ft = CAN_FT_DATA;
    g_transmit_message.tx_ff = CAN_FF_STANDARD;
    g_transmit_message.tx_dlen = 8;

    g_transmit_message.tx_data[0] = 0;
    g_transmit_message.tx_data[1] = 0;
    g_transmit_message.tx_data[2] = 0;
    g_transmit_message.tx_data[3] = 0;
	  g_transmit_message.tx_data[4] = 0;
    g_transmit_message.tx_data[5] = 0;
    g_transmit_message.tx_data[6] = 0;
    g_transmit_message.tx_data[7] = 0;
//		u8 mailbox_number=0;
	for(u8 i=0;i<100;i++)
	{
		Delay_ms(1);
		can_message_transmit(CAN0, &g_transmit_message);
		can_message_transmit(CAN0, &g_transmit_message);		
		iErrorCmt=can_transmit_error_number_get(CAN0);//����������
		if(iErrorCmt>10)
		{
			iErrorCmt=iErrorCmt;
		}
		
	}
	
	return iErrorCmt;
}


//FD CAN TEST
u8 iCanFdSmt=0;
void FdCanSendTest(u32 canid)
{
		u32 i=0;
		u8 flag=0;
		can_trasnmit_message_struct g_transmit_message;
		can_trasnmit_message_struct g_transmit_message1;
	
	  can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &g_transmit_message);
    g_transmit_message.tx_sfid = canid;
    g_transmit_message.tx_efid = 0x00;
    g_transmit_message.tx_ft = CAN_FT_DATA;//����֡
    g_transmit_message.tx_ff = CAN_FF_STANDARD;//��׼֡
    g_transmit_message.tx_dlen = 8;//֡����
    g_transmit_message.fd_flag = CAN_FDF_FDFRAME;//CAN FD
    g_transmit_message.fd_brs = CAN_BRS_ENABLE;//�������л� 
    g_transmit_message.fd_esi = CAN_ESI_RECESSIVE;//CAN_ESI_DOMINANT;//;
																
	
		for(i=0;i<12;i++)
		{
			g_transmit_message.tx_data[i] = i;
		}
		g_transmit_message.tx_data[0]=iCanFdSmt++;
    g_transmit_message1=g_transmit_message;
		g_transmit_message1.tx_dlen = 12;
		g_transmit_message1.tx_sfid = 0x7e0;
		g_transmit_message1.tx_efid = 0x00;
		
   /* transmit message */
   u8 mailbox_number=can_message_transmit(CAN0, &g_transmit_message);
	// ClearFdFlag();//���־λ	

	 while(1)
	 {
		 flag=can_transmit_states(CAN0,mailbox_number);
		 if(flag==CAN_TRANSMIT_OK) break;
		 
//		 can_transmit_error_number_get(CAN0);//����������
//		 
//		 flag=can_error_get(CAN0);//CAN �����ѯ
//		 if(flag!=CAN_ERROR_NONE)
//		 {
//			 i++;
//		 }
		 
		 if(i>5000)
		 {//����Ϊ�� �ܻᷢ��������
			 i=0;
				FdCanInitTest();
			 mailbox_number=can_message_transmit(CAN0, &g_transmit_message1);
			 //break;
		 }
		 i++;
		 
	 }
		
	
}




/*!
    \brief      initialize CAN  FD function
    \param[out] none
    \retval     none
		����USB ������Ϣ ��ʼ�� CAN FD ����

*/
void CanFD_Usbconfig(void)
{
    can_parameter_struct CAN_InitSt;
		can_fdframe_struct can_fd_parameter; //CAN FD ����
		can_fd_tdc_struct can_fd_tdc_parameter;//	
	
     //GPIO
		can_gpio_config();
	
    can_struct_para_init(CAN_INIT_STRUCT, &CAN_InitSt);
    /* initialize CAN register */
    can_deinit(CAN0);
    
	
	
    /* initialize CAN parameters */
    CAN_InitSt.time_triggered = DISABLE;
    CAN_InitSt.auto_bus_off_recovery = DISABLE;
    CAN_InitSt.auto_wake_up = DISABLE;
	
//		ʹ���Զ��ط� ENABLE
//		�����Զ��ط� DISABLE
    CAN_InitSt.auto_retrans = ENABLE;//�����Զ����� �Ƿ��� 
    CAN_InitSt.rec_fifo_overwrite = DISABLE;
    CAN_InitSt.trans_fifo_order = DISABLE;
		if(Fd_Working_mode>2) Fd_Working_mode=0;
		
    CAN_InitSt.working_mode = Fd_Working_mode;  //0=����ģʽ  2=��Ĭģʽ(���ڼ���)



	
	//speed1 �ٲ���������
		CAN_InitSt.resync_jump_width=SJW;
		CAN_InitSt.time_segment_1=BS1;
		CAN_InitSt.time_segment_2=BS2;
		CAN_InitSt.prescaler=Brp;//BAUD=60m/((1+6+1)*15)  87.5%
		
		/* initialize CAN */
    can_init(CAN0, &CAN_InitSt);
		//can_frequency_set(CAN0, DEV_CAN_BAUD_RATE);
  
    //������ ��ʼ��
		can_struct_para_init(CAN_FD_FRAME_STRUCT, &can_fd_parameter);
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
		
    can_fd_tdc_parameter.tdc_filter = 0x04; 
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = FdMode;//�Ƿ�ISO
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;
    

    //����������������
		can_fd_parameter.data_resync_jump_width=DataSJW;
		can_fd_parameter.data_time_segment_1=DataBS1;
		can_fd_parameter.data_time_segment_2=DataBS2;
		can_fd_parameter.data_prescaler=DataBrp;//BAUD=60m/((1+6+1)*15)  
		can_fd_init(CAN0, &can_fd_parameter);
		
  //can_fd_frequency_set(CAN0, 5000000);//1M  �Զ������ʺ���

    /* configure CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}




//��ȡ��FD ֡ ����
u8 GetFdCanLen(u8 iLen)
{
	u8 iSum=iLen;
//#define FDCAN_DLC_BYTES_0  ((uint32_t)0x00000000U) /*!< 0 bytes data field  */
//#define FDCAN_DLC_BYTES_1  ((uint32_t)0x00010000U) /*!< 1 bytes data field  */
//#define FDCAN_DLC_BYTES_2  ((uint32_t)0x00020000U) /*!< 2 bytes data field  */
//#define FDCAN_DLC_BYTES_3  ((uint32_t)0x00030000U) /*!< 3 bytes data field  */
//#define FDCAN_DLC_BYTES_4  ((uint32_t)0x00040000U) /*!< 4 bytes data field  */
//#define FDCAN_DLC_BYTES_5  ((uint32_t)0x00050000U) /*!< 5 bytes data field  */
//#define FDCAN_DLC_BYTES_6  ((uint32_t)0x00060000U) /*!< 6 bytes data field  */
//#define FDCAN_DLC_BYTES_7  ((uint32_t)0x00070000U) /*!< 7 bytes data field  */
//#define FDCAN_DLC_BYTES_8  ((uint32_t)0x00080000U) /*!< 8 bytes data field  */
//#define FDCAN_DLC_BYTES_12 ((uint32_t)0x00090000U) /*!< 12 bytes data field */
//#define FDCAN_DLC_BYTES_16 ((uint32_t)0x000A0000U) /*!< 16 bytes data field */
//#define FDCAN_DLC_BYTES_20 ((uint32_t)0x000B0000U) /*!< 20 bytes data field */
//#define FDCAN_DLC_BYTES_24 ((uint32_t)0x000C0000U) /*!< 24 bytes data field */
//#define FDCAN_DLC_BYTES_32 ((uint32_t)0x000D0000U) /*!< 32 bytes data field */
//#define FDCAN_DLC_BYTES_48 ((uint32_t)0x000E0000U) /*!< 48 bytes data field */
//#define FDCAN_DLC_BYTES_64 ((uint32_t)0x000F0000U) /*!< 64 bytes data field */
	if(iLen==9) iSum=12;
	else if(iLen==10) iSum=16;
	else if(iLen==11) iSum=20;
	else if(iLen==12) iSum=24;
	else if(iLen==13) iSum=32;
	else if(iLen==14) iSum=48;
	else if(iLen==15) iSum=64;
	return iSum;
}

//�ʺϷ���֡ 10 /2X֡
// 15765 ��׼֡   FD CAN
//cmdaddr  02 10 C0 00 00 00 00 00   ;8������λ
//���ݳ��� 8
// �����ճ�ʱ
// iReFlag=0 ֻ������
// iReFlag=1 ��������
// ����1 ��֡����ĳ���
u8 Send_Frame_FDCAN15765_Mul(u8 *cmdaddr,u8 iReFlag)   //
{
	//
	
	u32 i=0;
	u8 iValueRe=0,ExFlag=0,pos=0;
  u32 id=0;
	uint8_t TransmitMailbox=0;


	if((cmdaddr[0]&0xF0)==0X80)//��չ֡
	{
//		iValueRe=Send_Frame_EXCAN15765_Mul(cmdaddr,iReFlag);
//		return iValueRe;
		ExFlag=1;
		pos=2;
		id=(cmdaddr[1]&0x0FF)<<24;
		id+=(cmdaddr[2]&0x0FF)<<16;
		id+=(cmdaddr[3]&0x0FF)<<8;
		id+=cmdaddr[4];
		iValueRe=5;
	}
	else
	{
		pos=0;
		id=cmdaddr[1]*256+cmdaddr[2];
		//if(id==0) id=0xfC00;
		iValueRe=3;
	}
	
	
	u8 Stollen=GetFdCanLen(cmdaddr[0]&0x0F);//CAN FD ����

	iValueRe+=Stollen;
	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(ExFlag==1)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = id/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;
	}
	else
	{
		TbufMege.tx_sfid = id/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

   TbufMege.tx_ft = CAN_FT_DATA;//����֡
   TbufMege.tx_dlen = Stollen;//֡����
   TbufMege.fd_flag = CAN_FDF_FDFRAME;//CAN FD
   TbufMege.fd_brs = CAN_BRS_ENABLE;//�������л� 
   TbufMege.fd_esi = CAN_ESI_DOMINANT;//CAN_ESI_DOMINANT;//;

  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+3+pos];
  }

	
	if((TbufMege.tx_data[0]&0x30)==0x20)//2X֡ ��Ҫ�ȴ�һ֡30֡
	{
		while(1)
		{
			//break;//��λ���Ѿ� �޸��� 30֡
			i++;
			if(i>5000)
			{
				return 0;
				//break;//500ms��ʱ
			}
			if(iCan30flag==0x80)
			{
				SendCan30Buf();//����30֡ ����λ��
				if(iCan30Sum!=0)//�ظ� �� 30 00 ʱ
				{ 					
					iCan30Sum--;
					if(iCan30Sum==0)
					{
						Delay_ms(5);//������ʱһ�� �ȴ���λ������ 30֡
						iCan30flag=0;//�Ѿ����˹涨�Ķ�֡֡��  iCan30flag��� �豸�ȴ���һ֡30
					}
				}				
				break;
			}
			Delay_us(100);
		}
		
	}
	else 	if((TbufMege.tx_data[0]&0x30)==0x10)//1x֡ ��һ֡��Ҫ �ȴ���֡��
	{
		iCan30flag=0;
	}
	u8 flag=0;
	//can_transmit_state_enum iFlag;
	TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
	while(1)
	{
		 flag=can_transmit_states(CAN0,TransmitMailbox);
		 if(flag==CAN_TRANSMIT_OK) break;

		 if(i>250)
		 {//����Ϊ�� �ܻᷢ��������
			 i=0;
			 CanFD_Usbconfig();//��ʼ��������
			 CAN_setAllfit();
			 //FdCanInitTest();
			 TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
			 //break;
		 }
		 Delay_us(1);
		 i++;
		 
	 }
//	//�ж��Ƿ��ͳɹ�

	
  return iValueRe;//��һ֡ �����ַ
}


//�ʺϷ���֡ 10 /2X֡
// 15765 ��׼֡   FD CAN
//cmdaddr  02 10 C0 00 00 00 00 00   ;8������λ
//���ݳ��� 8
// �����ճ�ʱ
// iReFlag=0 ֻ������
// iReFlag=1 ��������
// ����1 ��֡����ĳ���
u8 Send_Frame_FDCAN15765_MulEx(u8 *cmdaddr,u8 iReFlag)   //
{
	//
	
	u32 i=0;
	u8 iValueRe=0,ExFlag=0,pos=0;
  u32 id=0;
	uint8_t TransmitMailbox=0;


	if((cmdaddr[0]&0xF0)==0X80)//��չ֡
	{
//		iValueRe=Send_Frame_EXCAN15765_Mul(cmdaddr,iReFlag);
//		return iValueRe;
		ExFlag=1;
		pos=2;
		id=(cmdaddr[1]&0x0FF)<<24;
		id+=(cmdaddr[2]&0x0FF)<<16;
		id+=(cmdaddr[3]&0x0FF)<<8;
		id+=cmdaddr[4];
		iValueRe=5;
	}
	else
	{
		pos=0;
		id=cmdaddr[1]*256+cmdaddr[2];
		//if(id==0) id=0xfC00;
		iValueRe=3;
	}
	
	
	u8 Stollen=GetFdCanLen(cmdaddr[0]&0x0F);//CAN FD ����

	iValueRe+=Stollen;
	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(ExFlag==1)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = id/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;
	}
	else
	{
		TbufMege.tx_sfid = id/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

   TbufMege.tx_ft = CAN_FT_DATA;//����֡
   TbufMege.tx_dlen = Stollen;//֡����
   TbufMege.fd_flag = CAN_FDF_FDFRAME;//CAN FD
   TbufMege.fd_brs = CAN_BRS_ENABLE;//�������л� 
   TbufMege.fd_esi = CAN_ESI_DOMINANT;//CAN_ESI_DOMINANT;//;

  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+3+pos];
  }

	
	
	u8 flag=0,iCanSend=0;
//	can_transmit_state_enum iFlag;
	TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
	while(1)
	{
		 flag=can_transmit_states(CAN0,TransmitMailbox);
		 if(flag==CAN_TRANSMIT_OK) break;

		 if(i>250)
		 {//����Ϊ�� �ܻᷢ��������
			 i=0;
			 CanFD_Usbconfig();//��ʼ��������
			 CAN_setAllfit();
			 //FdCanInitTest();
			 TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
			 iCanSend++;
			 if(iCanSend>200)
				break;
		 }
		 Delay_us(1);
		 i++;
		 
	 }
//	//�ж��Ƿ��ͳɹ�

	
  return iValueRe;//��һ֡ �����ַ
}


//�ʺϷ���֡ 10 /2X֡
// 15765 ��׼֡
//cmdaddr  02 10 C0 00 00 00 00 00   ;8������λ
//���ݳ��� 8
// �����ճ�ʱ
// iReFlag=0 ֻ������
// iReFlag=1 ��������
// ����1 �ɹ� ���� 0 ʧ��
u8 Send_Frame_CAN15765_Mul(u8 *cmdaddr,u8 iReFlag)   //
{
	//
	
	u32 i=0;
	u8 ExFlag=0,pos=0;
  u32 id=0;
//	uint8_t TransmitMailbox=0;


//	if(iFdCanFlag==0x80)
//	{
//			return 	Send_Frame_FDCAN15765_Mul(cmdaddr,iReFlag);
//	}
	
	
	if((cmdaddr[0]&0xF0)==0X80)//��չ֡
	{
//		iValueRe=Send_Frame_EXCAN15765_Mul(cmdaddr,iReFlag);
//		return iValueRe;
		ExFlag=1;
		pos=2;
		id=(cmdaddr[1]&0x0FF)<<24;
		id+=(cmdaddr[2]&0x0FF)<<16;
		id+=(cmdaddr[3]&0x0FF)<<8;
		id+=cmdaddr[4];
	}
	else
	{
		pos=0;
		id=cmdaddr[1]*256+cmdaddr[2];
		//if(id==0) id=0xfC00;
	}
	
	
	
	
	u8 Stollen=cmdaddr[0]&0x0F;
	if(Stollen==0) return 1;
	
	if(Stollen>8)  Stollen=8;
	
	
//	
	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(ExFlag==1)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = id/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;
	}
	else
	{
		TbufMege.tx_sfid = id/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

	
	TbufMege.tx_ft = CAN_FT_DATA;  
  TbufMege.tx_dlen = Stollen;
  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+3+pos];
  }

	
	if((TbufMege.tx_data[0]&0x30)==0x20)//2X֡ ��Ҫ�ȴ�һ֡30֡
	{
		while(1)
		{
			//break;//��λ���Ѿ� �޸��� 30֡
			i++;
			if(i>5000)
			{
				return 0;
				//break;//500ms��ʱ
			}
			if(iCan30flag==0x80)
			{
				SendCan30Buf();//����30֡ ����λ��
				if(iCan30Sum!=0)//�ظ� �� 30 00 ʱ
				{ 
					iCan30Sum--;
					if(iCan30Sum==0) 
					{
						Delay_ms(5);
						iCan30flag=0;//�Ѿ����˹涨�Ķ�֡֡��  iCan30flag��� �豸�ȴ���һ֡30
					}
				}				
				break;
			}
			Delay_us(100);
		}
		
	}
	else 	if((TbufMege.tx_data[0]&0x30)==0x10)//1x֡ ��һ֡��Ҫ �ȴ���֡��
	{
		iCan30flag=0;
	}
	
//	can_transmit_state_enum iFlag;
	can_message_transmit(CAN0, &TbufMege);
//	//�ж��Ƿ��ͳɹ�
//	GetTimer6Cnt();//���
//		i=0;
//		while(1)
//	 {
//		 i++;
//		 iFlag=can_transmit_states(CAN0,TransmitMailbox);
//		 if(iFlag==CAN_TRANSMIT_OK) break;
//		 if(i>0xFFFFFF)
//		 {
//			 
//			 break;
//		 }
//		 tmp=GetTimer6CntEx();
//		 if(tmp>1000*500) break;
//		 
//	 }
	

	//Rtollen ����
	//RbufFrame �����
	
  return 1;
}


//�ʺϷ���֡ 10 /2X֡
// 15765 ��׼֡
//cmdaddr  02 10 C0 00 00 00 00 00   ;8������λ
//���ݳ��� 8
// �����ճ�ʱ
// iReFlag=0 ֻ������
// iReFlag=1 ��������
// ����1 �ɹ� ���� 0 ʧ��
u8 Send_Frame_CAN15765_MulEx(u8 *cmdaddr,u8 iReFlag)   //
{
	//
	
	u8 iValueRe=0,ExFlag=0,pos=0;
  u32 id=0;
//	uint8_t TransmitMailbox=0;


//	if(iFdCanFlag==0x80)
//	{
//			return 	Send_Frame_FDCAN15765_Mul(cmdaddr,iReFlag);
//	}
	
	
	if((cmdaddr[0]&0xF0)==0X80)//��չ֡
	{
//		iValueRe=Send_Frame_EXCAN15765_Mul(cmdaddr,iReFlag);
//		return iValueRe;
		ExFlag=1;
		pos=2;
		id=(cmdaddr[1]&0x0FF)<<24;
		id+=(cmdaddr[2]&0x0FF)<<16;
		id+=(cmdaddr[3]&0x0FF)<<8;
		id+=cmdaddr[4];
		iValueRe=5;
	}
	else
	{
		pos=0;
		id=cmdaddr[1]*256+cmdaddr[2];
		//if(id==0) id=0xfC00;
		iValueRe=3;
	}
	
	
	
	
	u8 Stollen=cmdaddr[0]&0x0F;
	if(Stollen==0) return 1;
	
	if(Stollen>8)  Stollen=8;
	iValueRe+=Stollen;
	
//	
	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(ExFlag==1)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = id/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;
	}
	else
	{
		TbufMege.tx_sfid = id/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

	
	TbufMege.tx_ft = CAN_FT_DATA;  
  TbufMege.tx_dlen = Stollen;
  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+3+pos];
  }

	
	
//	can_transmit_state_enum iFlag;
	can_message_transmit(CAN0, &TbufMege);
//	//�ж��Ƿ��ͳɹ�
//	GetTimer6Cnt();//���
//		i=0;
//		while(1)
//	 {
//		 i++;
//		 iFlag=can_transmit_states(CAN0,TransmitMailbox);
//		 if(iFlag==CAN_TRANSMIT_OK) break;
//		 if(i>0xFFFFFF)
//		 {
//			 
//			 break;
//		 }
//		 tmp=GetTimer6CntEx();
//		 if(tmp>1000*500) break;
//		 
//	 }
	

	//Rtollen ����
	//RbufFrame �����
	
  return iValueRe;
}



//���Ӵ���


//iCanId CAN ID  ,�Ѿ���λ�� FC00(7E0)
// 15765 ��׼֡
//cmdaddr[0]= ���ͳ��� 
//cmdaddr[1...] ���͵�����
//ע: iCanId>0xFFFF  ʱΪ��չ֡ 
// ����1 �ɹ� ���� 0 ʧ��
u8 SendISO15765Data(u8 *cmdaddr,u32 iCanId)   //
{
	//
	
	u32 i=0,tmp=0;

//  u32 id=0,iTempValue=0;
	uint8_t TransmitMailbox=0;


	u8 Stollen=cmdaddr[0]&0x0F;
	if(Stollen==0) return 1;
	
	if(Stollen>8)  Stollen=8;
	
	
//	
	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(iCanId>0Xffff)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = iCanId/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;
	}
	else
	{
		TbufMege.tx_sfid = iCanId/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

	
	TbufMege.tx_ft = CAN_FT_DATA;  
  TbufMege.tx_dlen = Stollen;
  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+1];
  }

	
	
	
	can_transmit_state_enum iFlag;
	TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
//	//�ж��Ƿ��ͳɹ�
	GetTimer6Cnt();//���
		i=0;
	 while(1)
	 {
		 i++;
		 iFlag=can_transmit_states(CAN0,TransmitMailbox);
		 if(iFlag==CAN_TRANSMIT_OK) break;
		 if(i>0xFFFFFF)
		 {
			 
			 break;
		 }
		 tmp=GetTimer6CntEx();
		 if(tmp>1000*500) break;
		 
	 }

  return 1;
}


//iCanId CAN ID  ,�Ѿ���λ�� FC00(7E0)
// 15765 ��׼֡
//cmdaddr[0]= ���ͳ��� 
//cmdaddr[1...] ���͵�����
//ע: iCanId>0xFFFF  ʱΪ��չ֡
//CAN FD Э�� ���ݳ��� 1~8��12 16��20��24��32��48��64 ��󳤶���64����������1~64 ֮������ⳤ��
// ����1 �ɹ� ���� 0 ʧ��
u8 CanFdSendISO15765Data(u8 *cmdaddr,u32 iCanId)   //
{
	//
	
	u32 i=0;
//	u8 iValueRe=0,ExFlag=0,pos=0;

	uint8_t TransmitMailbox=0;

	u8 Stollen=GetFdCanLen(cmdaddr[0]&0x0F);//CAN FD ����

	//��ʼ������
	can_trasnmit_message_struct   TbufMege;
	can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &TbufMege);	
	if(iCanId>0xFFFF)//��չ֡
	{
		TbufMege.tx_sfid = 0;
		TbufMege.tx_efid = iCanId/0X08;
		TbufMege.tx_ff = CAN_FF_EXTENDED;//
	}
	else
	{
		TbufMege.tx_sfid = iCanId/0X20;
		TbufMege.tx_efid = 0x00;
		TbufMege.tx_ff = CAN_FF_STANDARD;
	}

   TbufMege.tx_ft = CAN_FT_DATA;//����֡
   TbufMege.tx_dlen = Stollen;//֡����
   TbufMege.fd_flag = CAN_FDF_FDFRAME;//CAN FD
   TbufMege.fd_brs = CAN_BRS_ENABLE;//�������л� 
   TbufMege.fd_esi = CAN_ESI_DOMINANT;//CAN_ESI_DOMINANT;//;

  for(u8 Sidx = 0; Sidx < Stollen; Sidx ++) 
  {
			TbufMege.tx_data[Sidx] = cmdaddr[Sidx+1];
  }

	
	
	u8 flag=0;
	//can_transmit_state_enum iFlag;
	TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
	while(1)
	{
		 flag=can_transmit_states(CAN0,TransmitMailbox);
		 if(flag==CAN_TRANSMIT_OK) break;

		 if(i>250)
		 {//����Ϊ�� �ܻᷢ��������
			 i=0;
			 Delay_ms(1);
			 CanFD_config(can_500k,Data_1M);//���³�ʼ����
			 CAN_setAllfit();
			 //FdCanInitTest();
			 TransmitMailbox=can_message_transmit(CAN0, &TbufMege);
			 //break;
			 Delay_ms(10);
		 }
		 Delay_us(1);
		 i++;
		 
	 }

  return 1;//
}

