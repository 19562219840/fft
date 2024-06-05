#ifndef _ADC_DMA_Timer_H
#define _ADC_DMA_Timer_H
 
#include "sys.h" 
#include "STM32F4xx_ADC.h" 
//#define SAM_FRE        200000//����Ƶ��
#define SAM_FRE        1024//����Ƶ��
//#define ADC1_DMA_Size  25000 //��������
#define ADC1_DMA_Size  1024 //��������
extern u16 ADC1_ConvertedValue[ ADC1_DMA_Size ]; 
void ADC_GPIO_Init(void);
void TIM3_Config( u32 Fre );
void ADC_Config( void );
void ADC_DMA_Trig( u16 Size );
 
#endif
