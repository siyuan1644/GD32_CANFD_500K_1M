#ifndef __TIMER_H
#define __TIMER_H
#include "gd32C10x.h"

void TIM1_config(void);
void TIM2_config(void);
void TIM3_config(void);


void TIM5_config(void);//���ڲ�����ʱ 
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

void TIM6_config(void);//���ڲ���ʱ�� 
uint32_t GetTimer6Cnt(void);
uint32_t GetTimer6CntEx(void);

//VPWM TEST
void Time1PwmInit(void);
void TIM0_Cap_Init(void);//����K������Ĳ��� ����
void TIM1_Cap_Init(void);
void TIM8_Cap_Init(void);


void TIM1_CH0_Cap_Init(void);//PWM ����
void TIM1_CH1_Cap_Init(void);//VPW ����

void dma_config(void);//DMA
void DMA_TIM1_CH0_Cap_Init(void);
//extern uint32_t Dmabuffer[512];

#endif


