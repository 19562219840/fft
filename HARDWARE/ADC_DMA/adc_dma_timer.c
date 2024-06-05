#include "adc_dma_timer.h"
u16 ADC1_ConvertedValue[ ADC1_DMA_Size ];
 
//ADC��GPIO���ã�PB1 - ADC1_IN9 | PB0 - ADC1_IN8
void ADC_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE ); 
	
	//PB0 - ADC1_IN8   
	//PB1 - ADC1_IN9   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB , &GPIO_InitStructure );
}
 
 
//TIM3��ʼ������ΪADC�Ĳ�������Դ
//Fre��ADC����Ƶ��
void TIM3_Config( u32 Fre )
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	u32 MaxData;
	u16 div=1;	
	while( (SystemCoreClock/2/Fre/div)>65535 )
	{
		div++;
	}
	MaxData =  SystemCoreClock/2/Fre/div - 1;	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3 , ENABLE );			   //����ʱ��
	
	//TIM3ʱ���׼����
	TIM_TimeBaseStructure.TIM_Period = MaxData ;
	TIM_TimeBaseStructure.TIM_Prescaler = div-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM3 , &TIM_TimeBaseStructure );
	
	TIM_SelectOutputTrigger( TIM3 , TIM_TRGOSource_Update );
	TIM_ARRPreloadConfig( TIM3 , ENABLE );
	TIM_Cmd( TIM3 , ENABLE );							                        //ʹ��TIM3
}
 
 
#define ADC1_DR_ADDRESS  ((uint32_t)0x4001204C)          //ADC1 DR

//ADC-DMA����ʹ��
//Size�����δ����������
void ADC_DMA_Trig( u16 Size )                    
{
	DMA2_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);				//����
	DMA2_Stream0->NDTR = Size;                             //��������
	DMA_ClearITPendingBit( DMA2_Stream0 ,DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0 );
	ADC1->SR = 0;
	DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;//����
}	
 
 
//ADC��DMA����������
//ע��2000K����Ƶ�ʣ��ɼ�6000�����ݣ���Ҫ����3ms
void ADC_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	NVIC_InitTypeDef 	  NVIC_InitStructure;
 
	ADC_DeInit( );
	DMA_DeInit( DMA2_Stream0 );
	/* Enable ADCx, DMA and GPIO clocks ****************************************/ 
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2 , ENABLE ); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1 , ENABLE );
	//RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2 , ENABLE );
	
	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);     
 
	/* DMA2 Stream0 channel0 configuration **************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;				 //����Ŀ�ĵ�
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADC1_ConvertedValue;       //���Ŀ�ĵ�
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC1_DMA_Size;                              //DMA ��������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;														//��������޸ĳ�ѭ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;					//����
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	//����
	DMA_Init( DMA2_Stream0 , &DMA_InitStructure );
	DMA_Cmd( DMA2_Stream0 , DISABLE );
	
	DMA_ClearITPendingBit( DMA2_Stream0 ,DMA_IT_TCIF0 );
	DMA_ITConfig( DMA2_Stream0 , DMA_IT_TC , ENABLE );
 
	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;     //����ģʽ
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;		//2��Ƶ
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit( &ADC_CommonInitStructure );
 
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		//ѭ��ɨ�衣��ͨ��ʱ��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ʹ��ENABLE����Ϊʹ���Զ�����ת����ʹ��DISABLE����Ϊ����ת����ת��һ�κ�ֹͣ��Ҫ�ֶ����Ʋ���������ת����
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//�ⲿ����ѡ��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;       //TIM3��ΪADC�Ĵ���Դ
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;											//�Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion = 1;                                   //����ͨ�����г��ȣ�����ֻ��һ��ͨ��9������һ�����������ã�
	ADC_Init( ADC1, &ADC_InitStructure );
	
	//PB0 - ADC1_IN8 - AD_UT_OP_BW   
	//PB1 - ADC1_IN9 - AD_UT_OP  
//	ADC_RegularChannelConfig( ADC1 , ADC_Channel_8 , 1, ADC_SampleTime_3Cycles);     //PB0 - ADC1_IN8 - AD_UT_OP_BW
	ADC_RegularChannelConfig( ADC1 , ADC_Channel_9 , 1, ADC_SampleTime_3Cycles);     //PB1 - ADC1_IN9 - AD_UT_OP
 
	ADC_DMARequestAfterLastTransferCmd( ADC1 , ENABLE );
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);
}
 
 
//DMA�жϷ�������ݲ��������ݴ����������һЩ�жϱ�־λ
void DMA2_Stream0_IRQHandler( void )
{	
	DMA_ClearITPendingBit( DMA2_Stream0 ,DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0 );
	DMA2->HIFCR = 0xffff;
	DMA2->LIFCR = 0xffff;

}
