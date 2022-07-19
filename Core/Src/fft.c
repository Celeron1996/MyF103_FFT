/**
  ******************************************************************************
  * @file    fft.c
  * @author  ZhuSL
  * @brief   fft 相关处理函数
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

uint32_t FFT_ADCBuffer[FFT_NUMBER];		/*FFT处理的原始数据，ADC采集音频的数据*/
long FFT_InBuffer[FFT_NUMBER];				/*FFT处理输入的数据*/
long FFT_OutBuffer[FFT_NUMBER/2];			/*FFT处理输出的数据*/
long FFT_ResultBuffer[FFT_NUMBER/2];			/*对FFT_OutBuffer处理的最终数据*/

/*通过定时器更新事件来触发ADC通过DMA进行转换*/
#define FFT_START()	do{HAL_ADC_Start_DMA(&hadc1, FFT_ADCBuffer, FFT_NUMBER);HAL_TIM_Base_Start(&htim3);}while(0)

/*停止定时器计数、DMA传输、ADC转换*/
#define FFT_STOP()	do{HAL_ADC_Stop_DMA(&hadc1);HAL_TIM_Base_Stop(&htim3);}while(0)


#define FFT_TASK_PRIO				1				/*FFT任务优先级*/
#define FFT_TASK_STACK_SIZE	1024		/*FFT任务栈大小*/
TaskHandle_t FFTTask_Handler;				/*FFT任务句柄*/


static uint8_t FFT_Display_FallPointBuffer[128];		/*频谱显示中下落的点*/

void (*const fft_display_list[])(void) = {fft_display0, fft_display1, fft_display2, fft_display3, fft_display4};	/*定义一个函数指针数组，用于存储全部频谱显示函数地址*/

/*频谱显示用到的斜率表*/
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
		
			//除以32768再乘65536是为了符合浮点数计算规律
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
  * @brief  屏幕底部显示128条柱状频谱
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY0_DIVI	(4u)		/*对最终值进行分频，以适合屏幕的显示*/
void fft_display0(void)
{
	uint8_t x,length;
	
	oled_clear();

	for(x = 1; x < 128; x++)	/*第一个直流分量不要*/
	{
		length = FFT_ResultBuffer[x]/FFT_DISPLAY0_DIVI;
		if(length > 63)	/*落下的点占一个像素，所以不是64*/
		{
			length = 63;
		}
		/*绘制每一列的柱子*/
		oled_draw_vline(x, 0, length);

		/*绘制每一列的下落点*/
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
  * @brief  屏幕中间显示128条对称的柱子
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY1_DIVI	(4u)		/*对最终值进行分频，以适合屏幕的显示*/
void fft_display1(void)
{
	uint8_t x,length;

	oled_clear();

	for(x = 1; x < 128; x++)	/*第一个直流分量不要*/
	{
		length = FFT_ResultBuffer[x]/FFT_DISPLAY1_DIVI;
		if(length > 31)
		{
			length = 31;
		}
		/*绘制每一列的柱子*/
		oled_draw_vline(x, 32, length);
		oled_draw_vline(x, 32-length, length);
		
		/*绘制每一列的下落点*/
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
  * @brief  原点放射状效果
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY2_DIVI		(4u)		/*对最终值进行分频，以适合屏幕的显示*/
void fft_display2(void)
{
	uint8_t count, length;
	uint16_t x,y;				/*终点坐标*/
	double k;					/*线的斜率*/
	uint8_t index = 0;	/*表示当前在处理的象限*/
	
	oled_clear();

	count = 1;/*第一个直流分量不要*/
	while(index < 4)
	{
		for(; count < 32; count++)
		{
			k = FFT_Display_Slope_Table[count];
			length = FFT_ResultBuffer[count + 32*index]/FFT_DISPLAY2_DIVI;
			fft_get_xy(&x, &y, k, length);
			
			/*检查坐标点有没有超出屏幕外，因为原点不对称，所以最大值小于屏幕尺寸*/
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
  * @brief  曲线圆形律动效果
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY3_DIVI			(6u)		/*对最终值进行分频，以适合屏幕的显示*/
#define FFT_DISPLAY3_RADIUS		(16u)		/*圆形的半径*/
void fft_display3(void)
{
	uint8_t count, length;
	uint16_t x,y;				/*本次循环采集的坐标*/
	double k;					/*线的斜率*/
	uint8_t index = 0;	/*表示当前在处理的象限*/
	uint16_t last_x = 0;		/*上一次循环采集的坐标点*/
	uint16_t last_y = 0;		/*上一次循环采集的坐标点*/
	uint16_t first_x = 0;		/*第一个点，需要记录下来，最后一个点与其连接*/
	uint16_t first_y = 0;		/*第一个点，需要记录下来，最后一个点与其连接*/
	
	oled_clear();

	count = 1;/*第一个直流分量不要*/
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
			length = FFT_ResultBuffer[count + 32*index]/FFT_DISPLAY3_DIVI + FFT_DISPLAY3_RADIUS;	/*加上实心圆的半径*/
			fft_get_xy(&x, &y, k, length);
			
			/*检查坐标点有没有超出屏幕外，因为原点不对称，所以最大值小于屏幕尺寸*/
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
			
			if((count == 1)&&(index == 0))	/*给本次循环的第一个点赋值，并记录下来*/
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
  * @brief  曲线律动效果
  * @param  None
  * @retval None
  */
#define FFT_DISPLAY4_DIVI			(6u)		/*对最终值进行分频，以适合屏幕的显示*/
#define FFT_DISPLAY4_WIDTH		(2u)		/*每间隔多少个取一个点*/
void fft_display4(void)
{
	uint8_t count = 0, length;
	uint16_t x,y;				/*当前循环坐标坐标*/
	uint8_t flag = 0;		/*1:上半周，0，下半周*/
	uint16_t last_x = 0;													/*上一次循环采集的坐标点*/
	uint16_t last_y = 32;													/*上一次循环采集的坐标点*/
	
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
  * @brief  以原点为起始参考点，根据斜率和长度计算正比例函数的终点坐标
	* @param  Xout				计算得出的X坐标
	*	@param	Yout				计算得出的Y坐标
	*	@param	K						斜率
	*	@param	Length			长度
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
