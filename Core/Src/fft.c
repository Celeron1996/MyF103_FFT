/**
  ******************************************************************************
  * @file    fft.c
  * @author  ZhuSL
  * @brief   fft ��ش�����
  *
  * @attention
  *
  ******************************************************************************
  */

#include "fft.h"
#include "oled.h"

#include "math.h"
#include "stm32_dsp.h"
#include "table_fft.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

#define FFT_ADC	hadc1
#define FFT_TIM	htim3

uint32_t FFT_ADCBuffer[FFT_NUMBER];		/*FFT�����ԭʼ���ݣ�ADC�ɼ���Ƶ������*/
long FFT_InBuffer[FFT_NUMBER];				/*FFT�������������*/
long FFT_OutBuffer[FFT_NUMBER/2];			/*FFT�������������*/
long FFT_ResultBuffer[FFT_NUMBER/2];			/*��FFT_OutBuffer�������������*/

/*ͨ����ʱ�������¼�������ADCͨ��DMA����ת��*/
#define FFT_START()	do{HAL_ADC_Start_DMA(&hadc1, FFT_ADCBuffer, FFT_NUMBER);HAL_TIM_Base_Start(&htim3);}while(0)

/*ֹͣ��ʱ��������DMA���䡢ADCת��*/
#define FFT_STOP()	do{HAL_ADC_Stop_DMA(&hadc1);HAL_TIM_Base_Stop(&htim3);}while(0)


#define FFT_TASK_PRIO				1				/*FFT�������ȼ�*/
#define FFT_TASK_STACK_SIZE	1024		/*FFT����ջ��С*/
TaskHandle_t FFTTask_Handler;				/*FFT������*/


static uint8_t FFT_Display_FallPointBuffer[128];		/*Ƶ����ʾ������ĵ�*/

void (*const fft_display_list[])(void) = {fft_display0, fft_display1, fft_display2, fft_display3, fft_display4};	/*����һ������ָ�����飬���ڴ洢ȫ��Ƶ����ʾ������ַ*/

/*Ƶ����ʾ�õ���б�ʱ�*/
const float FFT_Display_Slope_Table[32] = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 0.60, 0.70, 1.00, 1.20, 1.40, 1.60, 1.80, 2.00,
																						2.40, 2.60, 2.80, 3.00, 3.40, 3.60, 4.00, 5.50, 6.00, 8.00, 12.00, 18.00, 26.00, 36.00, 46.00, 56.00};
																					
void fft_get_xy(uint16_t *Xout, uint16_t *Yout, float K, uint16_t Length);


/**
  * @brief  fft processor
  * @param  None
  * @retval None
  */
void fft_processor(void)
{
	uint16_t i;
	int16_t lX,lY;
	float X,Y,Mag;

	for(i = 0;i < FFT_NUMBER; i++)
	{
		FFT_InBuffer[i] = ((int16_t)(FFT_ADCBuffer[i])) << 16;
	}
	
	cr4_fft_256_stm32(FFT_OutBuffer, FFT_InBuffer, FFT_NUMBER);
	
	for(i=0; i<FFT_NUMBER/2; i++)
	{
			lX  = (FFT_OutBuffer[i] << 16) >> 16;
			lY  = (FFT_OutBuffer[i] >> 16);
		
			//����32768�ٳ�65536��Ϊ�˷��ϸ������������
			X = FFT_NUMBER * ((float)lX) / 32768;
			Y = FFT_NUMBER * ((float)lY) / 32768;
			Mag = sqrt(X * X + Y * Y)*1.0/ FFT_NUMBER;
			if(i == 0)	
					FFT_ResultBuffer[i] = (unsigned long)(Mag * 32768);
			else
					FFT_ResultBuffer[i] = (unsigned long)(Mag * 65536);
	}	
}


/**
  * @brief  FFT_ADC_ConvCpltCallback
  * @param  None
  * @retval None
  */
__INLINE void FFT_ADC_ConvCpltCallback(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	
	FFT_STOP();
	vTaskNotifyGiveFromISR(FFTTask_Handler, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);	
}


/**
  * @brief  fft_task
  * @param  pvParameters
  * @retval None
  */
void fft_task(void *pvParameters)
{
	uint32_t notify;
	void (*p_fun_display)(void);
	uint16_t counter = 0;
	uint8_t index = 0;
	
	p_fun_display = fft_display_list[index];
	
	FFT_START();
	
	while(1)
	{
		notify = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(notify != 0)
		{
			fft_processor();
			FFT_START();
			oled_cmd_set(DISABLE);
			(*p_fun_display)();
			oled_cmd_set(ENABLE);
			counter++;
			if(counter >= 200)
			{
				index++;
				if(index >= ((sizeof(fft_display_list))/(sizeof(void (*)(void)))))
				{
					index = 0;
				}
				for(counter = 0; counter < sizeof(FFT_Display_FallPointBuffer); counter++)
				{
					FFT_Display_FallPointBuffer[counter] = 0;
				}
				counter = 0;
				p_fun_display = fft_display_list[index];
			}
		}
	}
}


/**
  * @brief  ��Ļ�ײ���ʾ128����״Ƶ��
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY0_DIVI	(4u)		/*������ֵ���з�Ƶ�����ʺ���Ļ����ʾ*/
void fft_display0(void)
{
	uint8_t x,length;
	
	oled_clear();

	for(x = 1; x < 128; x++)	/*��һ��ֱ��������Ҫ*/
	{
		length = FFT_ResultBuffer[x]/FFT_DISPLAY0_DIVI;
		if(length > 63)	/*���µĵ�ռһ�����أ����Բ���64*/
		{
			length = 63;
		}
		/*����ÿһ�е�����*/
		oled_draw_vline(x, 0, length);

		/*����ÿһ�е������*/
		if(length > FFT_Display_FallPointBuffer[x])
		{
			FFT_Display_FallPointBuffer[x] = length;
		}
		else
		{
			oled_draw_vline(x, FFT_Display_FallPointBuffer[x], 1);
			if(FFT_Display_FallPointBuffer[x] > 0)
			{
				FFT_Display_FallPointBuffer[x]--;
			}
		}
	}
}


/**
  * @brief  ��Ļ�м���ʾ128���ԳƵ�����
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY1_DIVI	(4u)		/*������ֵ���з�Ƶ�����ʺ���Ļ����ʾ*/
void fft_display1(void)
{
	uint8_t x,length;

	oled_clear();

	for(x = 1; x < 128; x++)	/*��һ��ֱ��������Ҫ*/
	{
		length = FFT_ResultBuffer[x]/FFT_DISPLAY1_DIVI;
		if(length > 31)
		{
			length = 31;
		}
		/*����ÿһ�е�����*/
		oled_draw_vline(x, 32, length);
		oled_draw_vline(x, 32-length, length);
		
		/*����ÿһ�е������*/
		if(length > FFT_Display_FallPointBuffer[x])
		{
			FFT_Display_FallPointBuffer[x] = length;
		}
		else
		{
			oled_draw_vline(x, 32 + FFT_Display_FallPointBuffer[x], 1);
			oled_draw_vline(x, 31 - FFT_Display_FallPointBuffer[x], 1);
			if(FFT_Display_FallPointBuffer[x] > 0)
			{
				FFT_Display_FallPointBuffer[x]--;
			}
		}
	}
}


/**
  * @brief  ԭ�����״Ч��
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY2_DIVI		(4u)		/*������ֵ���з�Ƶ�����ʺ���Ļ����ʾ*/
void fft_display2(void)
{
	uint8_t count, length;
	uint16_t x,y;				/*�յ�����*/
	double k;					/*�ߵ�б��*/
	uint8_t index = 0;	/*��ʾ��ǰ�ڴ��������*/
	
	oled_clear();

	count = 1;/*��һ��ֱ��������Ҫ*/
	while(index < 4)
	{
		for(; count < 32; count++)
		{
			k = FFT_Display_Slope_Table[count];
			length = FFT_ResultBuffer[count + 32*index]/FFT_DISPLAY2_DIVI;
			fft_get_xy(&x, &y, k, length);
			
			/*����������û�г�����Ļ�⣬��Ϊԭ�㲻�Գƣ��������ֵС����Ļ�ߴ�*/
			if((x > 60)&&(k <= 0.5))
			{
				x = 60;
				y = (float)x*k;
			}
			else if((y > 30)&&(k > 0.5))
			{
				y = 30;
				x = (float)y/k;
			}

			switch(index)
			{
				case 0:oled_draw_line(64, 32, 64+x, 32+y);break;	
				case 1:oled_draw_line(64, 32, 64-x, 32+y);break;
				case 2:oled_draw_line(64, 32, 64-x, 32-y);break;
				case 3:oled_draw_line(64, 32, 64+x, 32-y);break;
				default:break;
			}
			
			length = (uint8_t)sqrt(x*x + y*y);
			
			if(length > FFT_Display_FallPointBuffer[count + 32*index])
			{
				FFT_Display_FallPointBuffer[count + 32*index] = length;
			}
			else
			{
				fft_get_xy(&x, &y, k, FFT_Display_FallPointBuffer[count + 32*index]);
				if(FFT_Display_FallPointBuffer[count + 32*index] > 0)
				{
					FFT_Display_FallPointBuffer[count + 32*index]--;
				}
				
				switch(index)
				{
					case 0:oled_draw_pixel(64+x, 32+y);break;	
					case 1:oled_draw_pixel(64-x, 32+y);break;
					case 2:oled_draw_pixel(64-x, 32-y);break;
					case 3:oled_draw_pixel(64+x, 32-y);break;
					default:break;
				}
			}
		}
		count = 0;
		index++;
	}
}


/**
  * @brief  ����Բ���ɶ�Ч��
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY3_DIVI			(6u)		/*������ֵ���з�Ƶ�����ʺ���Ļ����ʾ*/
#define FFT_DISPLAY3_RADIUS		(16u)		/*Բ�εİ뾶*/
void fft_display3(void)
{
	uint8_t count, length;
	uint16_t x,y;				/*����ѭ���ɼ�������*/
	double k;					/*�ߵ�б��*/
	uint8_t index = 0;	/*��ʾ��ǰ�ڴ��������*/
	uint16_t last_x = 0;		/*��һ��ѭ���ɼ��������*/
	uint16_t last_y = 0;		/*��һ��ѭ���ɼ��������*/
	uint16_t first_x = 0;		/*��һ���㣬��Ҫ��¼���������һ������������*/
	uint16_t first_y = 0;		/*��һ���㣬��Ҫ��¼���������һ������������*/
	
	oled_clear();

	count = 1;/*��һ��ֱ��������Ҫ*/
	while(index < 4)
	{
		for(; count < 32; count++)
		{
			switch(index)
			{
				case 0:k = FFT_Display_Slope_Table[count];break;	
				case 1:k = FFT_Display_Slope_Table[31-count];break;
				case 2:k = FFT_Display_Slope_Table[count];break;
				case 3:k = FFT_Display_Slope_Table[31-count];break;
				default:break;
			}
			length = FFT_ResultBuffer[count + 32*index]/FFT_DISPLAY3_DIVI + FFT_DISPLAY3_RADIUS;	/*����ʵ��Բ�İ뾶*/
			fft_get_xy(&x, &y, k, length);
			
			/*����������û�г�����Ļ�⣬��Ϊԭ�㲻�Գƣ��������ֵС����Ļ�ߴ�*/
			if((x > 60)&&(k <= 0.5))
			{
				x = 60;
				y = (float)x*k;
			}
			else if((y > 30)&&(k > 0.5))
			{
				y = 30;
				x = (float)y/k;
			}
			
			if((count == 1)&&(index == 0))	/*������ѭ���ĵ�һ���㸳ֵ������¼����*/
			{
				last_x = first_x = 64+x;
				last_y = first_y = 32+y;
			}
			
			switch(index)
			{
				case 0:
					oled_draw_line(last_x, last_y, 64+x, 32+y);
					last_x = 64+x;
					last_y = 32+y;
					break;	
				case 1:
					oled_draw_line(last_x, last_y, 64-x, 32+y);
					last_x = 64-x;
					last_y = 32+y;				
					break;
				case 2:
					oled_draw_line(last_x, last_y, 64-x, 32-y);
					last_x = 64-x;
					last_y = 32-y;				
					break;
				case 3:
					oled_draw_line(last_x, last_y, 64+x, 32-y);
					last_x = 64+x;
					last_y = 32-y;				
					break;
				default:
					break;
			}
		}
		count = 0;
		index++;
	}
	oled_draw_line(first_x, first_y, 64+x, 32-y);
}


/**
  * @brief  �����ɶ�Ч��
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY4_DIVI			(6u)		/*������ֵ���з�Ƶ�����ʺ���Ļ����ʾ*/
#define FFT_DISPLAY4_WIDTH		(2u)		/*ÿ������ٸ�ȡһ����*/
void fft_display4(void)
{
	uint8_t count = 0, length;
	uint16_t x,y;				/*��ǰѭ����������*/
	uint8_t flag = 0;		/*1:�ϰ��ܣ�0���°���*/
	uint16_t last_x = 0;													/*��һ��ѭ���ɼ��������*/
	uint16_t last_y = 32;													/*��һ��ѭ���ɼ��������*/
	
	oled_clear();

	while((count + FFT_DISPLAY4_WIDTH) < 128)
	{
		count += FFT_DISPLAY4_WIDTH;
		length = FFT_ResultBuffer[count]/FFT_DISPLAY4_DIVI;
		if(length > 31)
		{
			length = 31;
		}
		x = count;
		if(flag == 0)
		{
			y = 32 - length;
			flag = 1;
		}
		else
		{
			y = 32 + length;
			flag = 0;
		}
		
		oled_draw_line(last_x, last_y, x, y);
		last_x = x;
		last_y = y;
	}
}


/**
  * @brief  ��ԭ��Ϊ��ʼ�ο��㣬����б�ʺͳ��ȼ����������������յ�����
	* @param  Xout				����ó���X����
	*	@param	Yout				����ó���Y����
	*	@param	K						б��
	*	@param	Length			����
  * @retval None
  */
__STATIC_INLINE void fft_get_xy(uint16_t *Xout, uint16_t *Yout, float K, uint16_t Length)
{
	double temp;

	temp = sqrt((double)((Length*Length)/(1 + K*K)));
	
	*Xout = (uint16_t)temp;
	
	*Yout = (uint16_t)(K*temp);
}


/**
  * @brief  create fft stask
  * @param  None
  * @retval None
  */
void fft_task_create(void)
{
	xTaskCreate((TaskFunction_t)fft_task,
							(const char *)"fft_task",
							( configSTACK_DEPTH_TYPE)FFT_TASK_STACK_SIZE,
							(void *)NULL,
							(UBaseType_t)FFT_TASK_PRIO,
							(TaskHandle_t * )&FFTTask_Handler);
}
