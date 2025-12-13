/*
 * @Author: 星必尘Sguan
 * @Date: 2025-12-09 19:01:17
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-13 16:21:25
 * @FilePath: \demo_ButterFly\Software\Motor.c
 * @Description: 四驱电机底层代码实现
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Motor.h"

extern ADC_HandleTypeDef hadc1;
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
    if (u_q <= 0) {
        __HAL_TIM_SET_COMPARE(htim,CH1,(uint16_t)(Sguan.MOTOR.Period*(-u_q)));
        __HAL_TIM_SET_COMPARE(htim,CH2,0);
    }
    else {
        __HAL_TIM_SET_COMPARE(htim,CH1,0);
        __HAL_TIM_SET_COMPARE(htim,CH2,(uint16_t)(Sguan.MOTOR.Period*u_q));
    }
}


static void Motor_OpenControl(uint8_t Motor_Num,float u_q) {
    u_q = Value_Limit(u_q,1,-1);
    switch (Motor_Num) {
    case 0:
        if (Sguan.MOTOR.Motor_Dir0) {
            Motor_SetDuty(&htim3,TIM_CHANNEL_1,TIM_CHANNEL_2,-u_q);
        }
        else {
            Motor_SetDuty(&htim3,TIM_CHANNEL_1,TIM_CHANNEL_2,u_q);
        }
        break;
    case 1:
        if (Sguan.MOTOR.Motor_Dir1) {
            Motor_SetDuty(&htim1,TIM_CHANNEL_2,TIM_CHANNEL_1,-u_q);
        }
        else {
            Motor_SetDuty(&htim1,TIM_CHANNEL_2,TIM_CHANNEL_1,u_q);
        }
        break;
    case 2:
        if (Sguan.MOTOR.Motor_Dir2) {
            Motor_SetDuty(&htim3,TIM_CHANNEL_3,TIM_CHANNEL_4,-u_q);
        }
        else {
            Motor_SetDuty(&htim3,TIM_CHANNEL_3,TIM_CHANNEL_4,u_q);
        }
        break;
    case 3:
        if (Sguan.MOTOR.Motor_Dir3) {
            Motor_SetDuty(&htim1,TIM_CHANNEL_3,TIM_CHANNEL_4,-u_q);
        }
        else {
            Motor_SetDuty(&htim1,TIM_CHANNEL_3,TIM_CHANNEL_4,u_q);
        }
        break;
    default:
        break;
    }
}

// 速度单环运算
static void Motor_Velocity_Math(uint8_t Motor_Num,PID_STRUCT *vel,ENCODER_STRUCT *encoder) {
    vel->Ref = encoder->Target_Speed;
    vel->Fbk = encoder->Real_Speed;
    PID_Control(vel);
    Motor_OpenControl(Motor_Num,vel->Out);
}

// 位置单环运算
static void Motor_Position_Math(uint8_t Motor_Num,PID_STRUCT *pos,ENCODER_STRUCT *encoder) {
    pos->Ref = encoder->Target_Pos;
    pos->Fbk = encoder->Real_Pos;
    PID_Control(pos);
    Motor_OpenControl(Motor_Num,pos->Out);
}

// 位置-速度环
static void Motor_PosVel_Math(uint8_t Motor_Num,PID_STRUCT *pos,PID_STRUCT *vel,ENCODER_STRUCT *encoder) {
    encoder->Response_Count++;
    if (encoder->Response_Count == Sguan.MOTOR.Response_Num) {
        pos->Ref = encoder->Target_Pos;
        pos->Fbk = encoder->Real_Pos;
        PID_Control(pos);
        encoder->Response_Count = 0;
    }
    vel->Ref = pos->Out;
    vel->Fbk = encoder->Real_Speed;
    PID_Control(vel);
    Motor_OpenControl(Motor_Num,vel->Out);
}

// PID运算的实现函数
static void Motor_PID_Calculate(void) {
    if (Sguan.Control_mode == Velocity_SINGLE_MODE) {
        Motor_Velocity_Math(0,&Sguan.Motor0_Speed,&Sguan.Encoder0);
        Motor_Velocity_Math(1,&Sguan.Motor1_Speed,&Sguan.Encoder1);
        Motor_Velocity_Math(2,&Sguan.Motor2_Speed,&Sguan.Encoder2);
        Motor_Velocity_Math(3,&Sguan.Motor3_Speed,&Sguan.Encoder3);
    }
    if (Sguan.Control_mode == Position_SINGLE_MODE) {
        Motor_Position_Math(0,&Sguan.Motor0_Position,&Sguan.Encoder0);
        Motor_Position_Math(1,&Sguan.Motor1_Position,&Sguan.Encoder1);
        Motor_Position_Math(2,&Sguan.Motor2_Position,&Sguan.Encoder2);
        Motor_Position_Math(3,&Sguan.Motor3_Position,&Sguan.Encoder3);
    }
    if (Sguan.Control_mode == PosVel_DOUBLE_MODE) {
        Motor_PosVel_Math(0,&Sguan.Motor0_Position,&Sguan.Motor0_Speed,&Sguan.Encoder0);
        Motor_PosVel_Math(1,&Sguan.Motor1_Position,&Sguan.Motor1_Speed,&Sguan.Encoder1);
        Motor_PosVel_Math(2,&Sguan.Motor2_Position,&Sguan.Motor2_Speed,&Sguan.Encoder2);
        Motor_PosVel_Math(3,&Sguan.Motor3_Position,&Sguan.Motor3_Speed,&Sguan.Encoder3);
    }
}

// 参数设定函数
static void Sguan_ParameterSet(void) {
    // 电机运行模式
    Sguan.Control_mode = Velocity_SINGLE_MODE;
    // 电机参数设置
    Sguan.MOTOR.Encoder_Dir0 = 1;
    Sguan.MOTOR.Encoder_Dir1 = 0;
    Sguan.MOTOR.Encoder_Dir2 = 1;
    Sguan.MOTOR.Encoder_Dir3 = 0;
    Sguan.MOTOR.Motor_Dir0 = 1;
    Sguan.MOTOR.Motor_Dir1 = 0;
    Sguan.MOTOR.Motor_Dir2 = 1;
    Sguan.MOTOR.Motor_Dir3 = 0;
    Sguan.MOTOR.Response_Num = 5;
    Sguan.MOTOR.Period = 2000;
    Sguan.MOTOR.Sampl_accuracy = 4096;
    // 编码器结构体
    Sguan.Encoder0.Kalman_R_Filter = 10.0f;
    Sguan.Encoder0.Kalman_Q_Filter = 0.001f;
    Sguan.Encoder1.Kalman_R_Filter = 10.0f;
    Sguan.Encoder1.Kalman_Q_Filter = 0.001f;
    Sguan.Encoder2.Kalman_R_Filter = 10.0f;
    Sguan.Encoder2.Kalman_Q_Filter = 0.001f;
    Sguan.Encoder3.Kalman_R_Filter = 10.0f;
    Sguan.Encoder3.Kalman_Q_Filter = 0.001f;
    // Motor0电机PID
    Sguan.Motor0_Speed.Kp = 0.02f;
    Sguan.Motor0_Speed.Ki = 0.0001f;
    Sguan.Motor0_Speed.Kd = 0.0f;
    Sguan.Motor0_Speed.OutMax = 1.0f;
    Sguan.Motor0_Speed.OutMin = -1.0f;
    Sguan.Motor0_Speed.ErrMax = 0;
    Sguan.Motor0_Speed.ErrMin = 0;
    Sguan.Motor0_Speed.ErrLimltFlag = 0;
    Sguan.Motor0_Speed.D_Filter = 0.2f;

    Sguan.Motor0_Position.Kp = 0.15f;
    Sguan.Motor0_Position.Ki = 0.00000025f;
    Sguan.Motor0_Position.Kd = 0.0f;
    Sguan.Motor0_Position.OutMax = 1.0f;
    Sguan.Motor0_Position.OutMin = -1.0f;
    Sguan.Motor0_Position.ErrMax = 0;
    Sguan.Motor0_Position.ErrMin = 0;
    Sguan.Motor0_Position.ErrLimltFlag = 0;
    Sguan.Motor0_Position.D_Filter = 0.2f;
    // Motor1电机PID（速度）
    Sguan.Motor1_Speed.Kp = 0.02f;
    Sguan.Motor1_Speed.Ki = 0.0001f;
    Sguan.Motor1_Speed.Kd = 0.0f;
    Sguan.Motor1_Speed.OutMax = 1.0f;
    Sguan.Motor1_Speed.OutMin = -1.0f;
    Sguan.Motor1_Speed.ErrMax = 0;
    Sguan.Motor1_Speed.ErrMin = 0;
    Sguan.Motor1_Speed.ErrLimltFlag = 0;
    Sguan.Motor1_Speed.D_Filter = 0.2f;
    // 位置
    Sguan.Motor1_Position.Kp = 0.15f;
    Sguan.Motor1_Position.Ki = 0.00000025f;
    Sguan.Motor1_Position.Kd = 0.0f;
    Sguan.Motor1_Position.OutMax = 1.0f;
    Sguan.Motor1_Position.OutMin = -1.0f;
    Sguan.Motor1_Position.ErrMax = 0;
    Sguan.Motor1_Position.ErrMin = 0;
    Sguan.Motor1_Position.ErrLimltFlag = 0;
    Sguan.Motor1_Position.D_Filter = 0.2f;
    // Motor2电机PID（速度）
    Sguan.Motor2_Speed.Kp = 0.02f;
    Sguan.Motor2_Speed.Ki = 0.0001f;
    Sguan.Motor2_Speed.Kd = 0.0f;
    Sguan.Motor2_Speed.OutMax = 1.0f;
    Sguan.Motor2_Speed.OutMin = -1.0f;
    Sguan.Motor2_Speed.ErrMax = 0;
    Sguan.Motor2_Speed.ErrMin = 0;
    Sguan.Motor2_Speed.ErrLimltFlag = 0;
    Sguan.Motor2_Speed.D_Filter = 0.2f;
    // 位置
    Sguan.Motor2_Position.Kp = 0.15f;
    Sguan.Motor2_Position.Ki = 0.00000025f;
    Sguan.Motor2_Position.Kd = 0.0f;
    Sguan.Motor2_Position.OutMax = 1.0f;
    Sguan.Motor2_Position.OutMin = -1.0f;
    Sguan.Motor2_Position.ErrMax = 0;
    Sguan.Motor2_Position.ErrMin = 0;
    Sguan.Motor2_Position.ErrLimltFlag = 0;
    Sguan.Motor2_Position.D_Filter = 0.2f;
    // Motor3电机PID（速度）
    Sguan.Motor3_Speed.Kp = 0.02f;
    Sguan.Motor3_Speed.Ki = 0.0001f;
    Sguan.Motor3_Speed.Kd = 0.0f;
    Sguan.Motor3_Speed.OutMax = 1.0f;
    Sguan.Motor3_Speed.OutMin = -1.0f;
    Sguan.Motor3_Speed.ErrMax = 0;
    Sguan.Motor3_Speed.ErrMin = 0;
    Sguan.Motor3_Speed.ErrLimltFlag = 0;
    Sguan.Motor3_Speed.D_Filter = 0.2f;
    // 位置
    Sguan.Motor3_Position.Kp = 0.15f;
    Sguan.Motor3_Position.Ki = 0.00000025f;
    Sguan.Motor3_Position.Kd = 0.0f;
    Sguan.Motor3_Position.OutMax = 1.0f;
    Sguan.Motor3_Position.OutMin = -1.0f;
    Sguan.Motor3_Position.ErrMax = 0;
    Sguan.Motor3_Position.ErrMin = 0;
    Sguan.Motor3_Position.ErrLimltFlag = 0;
    Sguan.Motor3_Position.D_Filter = 0.2f;
}

// 电机初始化
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
    // 读取初始ADC，获得Encoder偏置(如果需要)
    #ifdef DEBUG
    Sguan.Encoder0.Offset = Sguan.Encoder0.Raw_data;
    Sguan.Encoder1.Offset = Sguan.Encoder1.Raw_data;
    Sguan.Encoder2.Offset = Sguan.Encoder2.Raw_data;
    Sguan.Encoder3.Offset = Sguan.Encoder3.Raw_data;
    #endif // DEBUG
    // 电机参数设定
    Sguan_ParameterSet();
}

// 电机中断服务函数
void Motor_RunTick(void) {
    // 再次开启ADC采样
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // 速度测量
    Encoder_RunTick();
    // PID运算并更新电机
    Motor_PID_Calculate();
}

