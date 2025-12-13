/*
 * @Author: 星必尘Sguan
 * @Date: 2025-12-09 23:26:21
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-12 20:47:43
 * @FilePath: \demo_ButterFly\Hardware\Encoder.c
 * @Description: 编码器底层函数编写
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Encoder.h"
// 外部声明
#include "filter.h"
#include "Motor.h"

// 重写fmodf函数
static float Sguan_fmodf(float x, float y) {
  if (y == 0.0f) return 0.0f;
  int quotient = (int)(x / y); // 1次除法运算
  return x - quotient * y;     // 1次乘1次减
}

// 参数限定0-2pi
static float normalize_angle(float angle) {
  float normalized = Sguan_fmodf(angle, Value_PI*2);
  // 如果结果为负，加上2π使其在[0, 2π)范围内
  if (normalized < 0) {
      normalized += Value_PI*2;
  }
  return normalized;
}

// 获取实际电机角度值
static float Encoder_GetRad(ENCODER_STRUCT *encoder) {
	return normalize_angle((encoder->Raw_data - encoder->Offset)*2*Value_PI/Sguan.MOTOR.Sampl_accuracy);
}

// 获取实际电机多圈角度
static float Encoder_GetPos(ENCODER_STRUCT *encoder) {
    return Encoder_GetRad(encoder) + encoder->PosFlag*Value_PI*2;
}

// 获取实际电机速度(未滤波)
static void Encoder_ReadSpeed(ENCODER_STRUCT *encoder,float *real_speed) {
    float This_Count = Encoder_GetRad(encoder);
    uint32_t This_Time = HAL_GetTick();
    float Encoder_num =  This_Count - encoder->LastRad;
    int32_t Encoder_time = This_Time - encoder->LastTick;
    if (Encoder_num >= Value_PI*1.2f) {
        Encoder_num -= Value_PI*2;
        encoder->PosFlag--;
    }
    if (Encoder_num <= -Value_PI*1.2f) {
        Encoder_num += Value_PI*2;
        encoder->PosFlag++;
    }
    if (Encoder_time) {
        *real_speed = (Encoder_num*1000)/Encoder_time;
    }
    encoder->LastRad = This_Count;
    encoder->LastTick = This_Time;
}

// 获取实际电机速度(已滤波)
static float Encoder_FilterSpeed(ENCODER_STRUCT *encoder,uint8_t Motor_CH) {
    static float data = 0;
    Encoder_ReadSpeed(encoder,&data);
    return kalman_filter_std(Motor_CH, data, encoder->Kalman_R_Filter, encoder->Kalman_Q_Filter);
}

// Encoder运行Tick
void Encoder_RunTick(void) {
	if (Sguan.MOTOR.Encoder_Dir0) {
		Sguan.Encoder0.Real_Speed = -Encoder_FilterSpeed(&Sguan.Encoder0,0);
		Sguan.Encoder0.Real_Pos = -Encoder_GetPos(&Sguan.Encoder0);
	}
	else {
		Sguan.Encoder0.Real_Speed = Encoder_FilterSpeed(&Sguan.Encoder0,0);
		Sguan.Encoder0.Real_Pos = Encoder_GetPos(&Sguan.Encoder0);
	}
	if (Sguan.MOTOR.Encoder_Dir1) {
		Sguan.Encoder1.Real_Speed = -Encoder_FilterSpeed(&Sguan.Encoder1,1);
		Sguan.Encoder1.Real_Pos = -Encoder_GetPos(&Sguan.Encoder1);
	}
	else {
		Sguan.Encoder1.Real_Speed = Encoder_FilterSpeed(&Sguan.Encoder1,1);
		Sguan.Encoder1.Real_Pos = Encoder_GetPos(&Sguan.Encoder1);
	}
	if (Sguan.MOTOR.Encoder_Dir2) {
		Sguan.Encoder2.Real_Speed = -Encoder_FilterSpeed(&Sguan.Encoder2,2);
		Sguan.Encoder2.Real_Pos = -Encoder_GetPos(&Sguan.Encoder2);
	}
	else {
		Sguan.Encoder2.Real_Speed = Encoder_FilterSpeed(&Sguan.Encoder2,2);
		Sguan.Encoder2.Real_Pos = Encoder_GetPos(&Sguan.Encoder2);
	}
	if (Sguan.MOTOR.Encoder_Dir3) {
		Sguan.Encoder3.Real_Speed = -Encoder_FilterSpeed(&Sguan.Encoder3,3);
		Sguan.Encoder3.Real_Pos = -Encoder_GetPos(&Sguan.Encoder3);
	}
	else {
		Sguan.Encoder3.Real_Speed = Encoder_FilterSpeed(&Sguan.Encoder3,3);
		Sguan.Encoder3.Real_Pos = Encoder_GetPos(&Sguan.Encoder3);
	}
}

// ADC采样完成回调函数
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {		
	Sguan.Encoder0.Raw_data = ADC1->JDR1;
	Sguan.Encoder1.Raw_data = ADC1->JDR2;
	Sguan.Encoder2.Raw_data = ADC1->JDR3;
 	Sguan.Encoder3.Raw_data = ADC1->JDR4;
}

