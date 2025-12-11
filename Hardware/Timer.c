/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-08 19:26:48
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-12 00:40:35
 * @FilePath: \demo_ButterFly\Hardware\Timer.c
 * @Description: TIM定时中断统一管理函数;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Timer.h"
#include "adc.h"
// 常量宏定义
#define Value_PI 3.14159265358979323846f

/**
 * @description: TIM2中断回调函数，1ms的定时器定时中断;
 * @param {TIM_HandleTypeDef*} htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {  // 1kHz中断
        HAL_ADCEx_InjectedStart_IT(&hadc1);
    }
}




