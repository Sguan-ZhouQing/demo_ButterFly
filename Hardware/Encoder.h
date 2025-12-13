#ifndef __ENCODER_H
#define __ENCODER_H

#include "adc.h"
// 常量宏定义
#define Value_PI 3.14159265358979323846f

typedef struct{
    // 多圈角度计算and偏置
    int32_t PosFlag;        // 记录多圈圈数
    float Offset;           // 编码器偏置
    uint8_t Response_Count; // 记录PID内外环响应次数
    // Real-data
    uint16_t Raw_data;      // 实际的原始ADC采样数据
    float Real_Speed;       // 实际的速度(Real)
    float Real_Pos;         // 实际的多圈角度(Real)
    // Target-data
    float Target_Speed;     // 期望的“实时”运行速度(Target)
    float Target_Pos;       // 期望的“实时”运行位置(Target)
    // 计算速度和滤波
    float LastRad;          // 记录的角度
    uint32_t LastTick;      // 记录的上次时间
    float Kalman_R_Filter;  // 噪声越大，参数越大
    float Kalman_Q_Filter;  // 系统反应越快，参数越大
}ENCODER_STRUCT;

// 函数声明
void Encoder_RunTick(void);

#endif // ENCODER_H
