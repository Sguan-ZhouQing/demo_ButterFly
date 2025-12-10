/*
 * @Author: 星必尘Sguan
 * @Date: 2025-12-09 19:01:17
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-10 21:00:59
 * @FilePath: \demo_ButterFly\Software\Motor.c
 * @Description: 四驱电机底层代码实现
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Motor.h"
// 参数限制函数
#define Value_Limit(val, max, min) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

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


void Motor_Init(void) {
    
}


