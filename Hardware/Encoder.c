/*
 * @Author: 星必尘Sguan
 * @Date: 2025-12-09 23:26:21
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-10 20:59:34
 * @FilePath: \demo_ButterFly\Hardware\Encoder.c
 * @Description: 编码器
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Encoder.h"

uint16_t ADC_InjectedValues[4];

void Encoder_ReadSpeed_Loop(void) {
    
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {		
	ADC_InjectedValues[0] = ADC1->JDR1;
	ADC_InjectedValues[1] = ADC1->JDR2;
	ADC_InjectedValues[2] = ADC1->JDR3;
 	ADC_InjectedValues[3] = ADC1->JDR4;
}

