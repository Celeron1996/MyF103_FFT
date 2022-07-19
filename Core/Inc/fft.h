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
	
#ifndef __FFT_H
#define __FFT_H

#include "stm32f1xx_hal.h"


#define FFT_NUMBER	(256u)




void fft_processor(void);
void FFT_ADC_ConvCpltCallback(void);
void fft_task(void *pvParameters);
void fft_display0(void);
void fft_display1(void);
void fft_display2(void);
void fft_display3(void);
void fft_display4(void);
void fft_task_create(void);


#endif
