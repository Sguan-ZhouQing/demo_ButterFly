#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "Encoder.h"
#include "nRF_printf.h"

// 参数限制函数
#define Value_Limit(val, max, min) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))
// 编码器Offset
// #define DEBUG

typedef enum{
    Velocity_SINGLE_MODE=0, // 速度单闭环(速度控制)
    Position_SINGLE_MODE,   // 位置单闭环(角度控制)
    PosVel_DOUBLE_MODE,     // 位置-速度串级PID
}MOTOR_MODE_ENUM;

typedef struct{
    // 编码器
    uint8_t Encoder_Dir0;   // 编码器旋转方向
    uint8_t Encoder_Dir1;   // 编码器旋转方向
    uint8_t Encoder_Dir2;   // 编码器旋转方向
    uint8_t Encoder_Dir3;   // 编码器旋转方向
    // 电机
    uint8_t Motor_Dir0;     // 电机旋转方向
    uint8_t Motor_Dir1;     // 电机旋转方向
    uint8_t Motor_Dir2;     // 电机旋转方向
    uint8_t Motor_Dir3;     // 电机旋转方向
    // PID响应
    uint8_t Response_Count; // 记录内外环响应次数
    uint8_t Response_Num;   // PID内外环响应倍数
    // 占空比设定
    uint16_t Period;        // 满占空比Period数值
    uint16_t Sampl_accuracy;// ADC采样精度 
    // 运行参数
    float Target_Speed;     // 期望的“实时”运行速度(Target)
    float Target_Pos;       // 期望的“实时”运行位置(Target)
}MOTOR_STRUCT;

typedef struct{
	float Kp;	   		    // 参数: 比例环路增益
	float Ki;	   		    // 参数: 积分增益	
	float Kd;           	// 参数: 微分系数
	float Ref;	  		  	// 输入: 参考设定点
	float Fbk;	 		    // 输入: 反馈值
	float Out;	   	    	// 输出: 控制器输出
	float Err;  		    // 数据; 误差
	float ErrLast;      	// 数据: 上次误差
  	float Ui;	          	// 数据: 积分项
	float OutMax; 		  	// 参数: 上限饱和限制
	float OutMin;		    // 参数: 下限饱和限制
	float ErrMax;     		// 参数: 误差上限
	float ErrMin;     		// 参数: 误差下限	
	float ErrLimltFlag;		// 参数: 误差限幅标志
	float D_Filter;		    // 参数: 微分项滤波系数 (new)
	float LastDerivative;   // 数据: 上次微分项值 (new)
}PID_STRUCT;

typedef struct{
    MOTOR_MODE_ENUM Control_mode;   // 电机运行模式

    MOTOR_STRUCT MOTOR;             // 电机参数设置

    ENCODER_STRUCT Encoder0;        // 编码器结构体
    ENCODER_STRUCT Encoder1;        // 编码器结构体
    ENCODER_STRUCT Encoder2;        // 编码器结构体
    ENCODER_STRUCT Encoder3;        // 编码器结构体
    
    PID_STRUCT Motor0_Speed;        // 电机PID
    PID_STRUCT Motor0_Position;     // 电机PID
    PID_STRUCT Motor1_Speed;        // 电机PID
    PID_STRUCT Motor1_Position;     // 电机PID
    PID_STRUCT Motor2_Speed;        // 电机PID
    PID_STRUCT Motor2_Position;     // 电机PID
    PID_STRUCT Motor3_Speed;        // 电机PID
    PID_STRUCT Motor3_Position;     // 电机PID

    NRF24_STRUCT nRF24;             // nRF无线透传
}Motor_System_STRUCT;

// 结构体声明
extern Motor_System_STRUCT Sguan;
// 函数声明
void Motor_Init(void);
void Motor_RunTick(void);

#endif // MOTOR_H
