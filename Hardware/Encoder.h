#ifndef __ENCODER_H
#define __ENCODER_H

#include "adc.h"

typedef struct{
    int32_t PosFlag;        // 记录多圈圈数
    float Offset;           // 编码器偏置
    float Real_Speed;       // 实际的速度(Real)
    float Real_Pos;         // 实际的多圈角度(Real)
    float Real_El_Rad;      // 实际的单圈角度(Real)
    float LastRad;          // 记录的角度
    uint32_t LastTick;      // 记录的上次时间
    float Kalman_R_Filter;  // 噪声越大，参数越大
    float Kalman_Q_Filter;  // 系统反应越快，参数越大
}ENCODER_STRUCT;

void Encoder_ReadSpeed_Loop(void);



#endif // ENCODER_H
