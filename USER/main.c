#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h"
#include "adc_dma_timer.h"
#include "key.h"
#include "timer.h" 
#include "math.h" 
#include "arm_math.h"  

//ALIENTEK 探索者STM32F407开发板 实验13
//LCD显示实验-库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK
#define FFT_LENGTH		1024 		//FFT长度，默认是1024点FFT

float fft_inputbuf[FFT_LENGTH*2];	//FFT输入数组
float fft_outputbuf[FFT_LENGTH];	//FFT输出数组
u32 i=0;
u8 timeout;//定时器溢出次数

int main(void)
{  
  arm_cfft_radix4_instance_f32 scfft;
 	u8 key,t=0;
	float time; 
	u8 buf[50]; 
	u16 i; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					//初始化LED
	KEY_Init();					//初始化按键
 	//LCD_Init();					//初始化LCD
	//u8 lcd_id[12];				//存放LCD ID字符串
	ADC_GPIO_Init();						    // ADC引脚初始化。
	TIM3_Config( SAM_FRE );                     // 触发ADC采样频率，采样频率2MHz
	ADC_Config();                               // ADC1024采样频率，采集1024个数据
	TIM2_Int_Init(65535,84-1);	//1Mhz计数频率,最大计时65ms左右超出

	/*POINT_COLOR=RED;      //画笔颜色：红色
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"DSP FFT TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/7/2");	
	LCD_ShowString(30,130,200,16,16,"KEY0:Run FFT");//显示提示信息 
	LCD_ShowString(30,160,200,16,16,"FFT runtime:");//显示FFT执行时间*/

		arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数
		
	//sprintf((char*)lcd_id,"LCD ID:%04X",lcddev.id);//将LCD ID打印到lcd_id数组。				 	
  	while(1) 
	{		 

		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{			 ADC_DMA_Trig( ADC1_DMA_Size );          // 开始AD采集，设置采样点数
           delay_ms(2000);                            // 延时3ms，等待ADC数据全部转换到 ADC1_ConvertedValue数组中
					 delay_ms(2000);                            // 延时3ms，等待ADC数据全部转换到 ADC1_ConvertedValue数组中

			for(i=0;i<FFT_LENGTH;i++)//生成信号序列
			{
				 //fft_inputbuf[2*i] = (float)ADC1_ConvertedValue[ i ]*3.3f/4096.0f;//实部为ADC采样值
				 				 fft_inputbuf[2*i]=100+
				                   10*arm_sin_f32(2*PI*i/FFT_LENGTH)+
								           30*arm_sin_f32(2*PI*i*4/FFT_LENGTH)+
				                   50*arm_cos_f32(2*PI*i*8/FFT_LENGTH);	//生成输入信号实部

				fft_inputbuf[2*i+1]=0;//虚部全部为0
			}
			TIM_SetCounter(TIM2,0);//重设TIM3定时器的计数器值
			timeout=0;
			arm_cfft_radix4_f32(&scfft,fft_inputbuf);	//FFT计算（基4）
			time=TIM_GetCounter(TIM2)+(u32)timeout*65536; 			//计算所用时间
			sprintf((char*)buf,"%0.3fms\r\n",time/1000);	
			//LCD_ShowString(30+12*8,160,100,16,16,buf);	//显示运行时间		
			arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值 
			printf("\r\n%d point FFT runtime:%0.3fms\r\n",FFT_LENGTH,time/1000);
			printf("FFT Result:\r\n");
			for(i=0;i<FFT_LENGTH;i++)
			{
				printf("fft_outputbuf[%d]:%f\r\n",i,fft_outputbuf[i]);
			}
			/*for(i=0;i<FFT_LENGTH;i++)
			{
				printf("ADC1_ConvertedValue[%d]:%f\r\n",i,ADC1_ConvertedValue[ i ]*3.3f/4096.0f);
			}*/

		}else delay_ms(10);
		t++;
		if((t%10)==0)LED0=!LED0;		  

		}
}

