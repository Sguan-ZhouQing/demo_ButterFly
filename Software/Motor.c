/*
 * @Author: 星必尘Sguan
 * @Date: 2025-12-09 19:01:17
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-12 01:17:53
 * @FilePath: \demo_ButterFly\Software\Motor.c
 * @Description: 四驱电机底层代码实现
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
Motor_System_STRUCT Sguan;

/**
 * @description: PID计算抗积分饱和（带微分项滤波）
 * @param {PID_STRUCT} *p
 * @return {*}
 */
static void PID_Control(PID_STRUCT *p) {
    p->Err = p->Ref - p->Fbk;
    // 误差限幅
    if(p->ErrLimltFlag == 1) {
        p->Err = Value_Limit(p->Err, p->ErrMax, p->ErrMin);
    }
    // 积分抗饱和
    if(p->Out != p->OutMax && p->Out != p->OutMin) {
        p->Ui += p->Err;
    }
    
    // 微分项计算（带低通滤波）
    float derivative_term = p->Err - p->ErrLast;
    // 应用低通滤波：D_filter通常在0.1~0.9之间，值越小滤波效果越强
    if(p->D_Filter > 0) {
        derivative_term = p->D_Filter * derivative_term + (1 - p->D_Filter) * p->LastDerivative;
        p->LastDerivative = derivative_term;
    }

    // PID计算
    p->Out = p->Kp * p->Err + p->Ki * p->Ui + p->Kd * derivative_term;
    p->ErrLast = p->Err;
    // 输出限幅
    p->Out = Value_Limit(p->Out, p->OutMax, p->OutMin);
}


static void Motor_SetDuty(TIM_HandleTypeDef *htim,uint32_t CH1,uint32_t CH2,float u_q) {
    if (u_q >= 0) {
        __HAL_TIM_SET_COMPARE(htim,CH1,(uint16_t)(500*u_q));
        __HAL_TIM_SET_COMPARE(htim,CH2,0);
    }
    else {
        __HAL_TIM_SET_COMPARE(htim,CH1,0);
        __HAL_TIM_SET_COMPARE(htim,CH2,(uint16_t)(500*u_q));
    }
}


static void Motor_OpenControl(uint8_t Motor_Num,float u_q) {
    switch (Motor_Num) {
    case 0:
        Motor_SetDuty(&htim3,TIM_CHANNEL_1,TIM_CHANNEL_2,u_q);
        break;
    case 1:
        Motor_SetDuty(&htim1,TIM_CHANNEL_2,TIM_CHANNEL_1,u_q);
        break;
    case 2:
        Motor_SetDuty(&htim3,TIM_CHANNEL_3,TIM_CHANNEL_4,u_q);
        break;
    case 3:
        Motor_SetDuty(&htim1,TIM_CHANNEL_3,TIM_CHANNEL_4,u_q);
        break;
    default:
        break;
    }
}


static void Motor_CloseRun_Loop(void) {

}



void Motor_Init(void) {
    // PWM启用初始化
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
