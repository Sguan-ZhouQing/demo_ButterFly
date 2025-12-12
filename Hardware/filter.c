/*
 * @Author: 星必尘Sguan
 * @Date: 2025-08-29 14:27:38
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-12-12 20:43:38
 * @FilePath: \demo_ButterFly\Hardware\filter.c
 * @Description: 底层滤波函数代码编写
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "filter.h"

// 低通滤波(alpha越小，滤波越好)
float low_pass_filter(float input, float last_output, float alpha) {
  return alpha * input + (1.0f - alpha) * last_output;
}

// 卡尔曼滤波(r越大，越抑制噪声|q越大，越适应快速响应)
float kalman_filter_std(uint8_t Motor_CH, float input, float r, float q) {
  static float z_CH0;
  static float p_CH0 = 1;
  static float z_CH1;
  static float p_CH1 = 1;
  static float z_CH2;
  static float p_CH2 = 1;
  static float z_CH3;
  static float p_CH3 = 1;
  float z,g;
  switch (Motor_CH)
  {
  case 0:
    p_CH0 = p_CH0 + q;
    g = p_CH0 / (p_CH0 + r);
    z_CH0 = z_CH0 + g * (input - z_CH0);
    p_CH0 = (1 - g) * p_CH0;
    z = z_CH0;
    break;
  case 1:
    p_CH1 = p_CH1 + q;
    g = p_CH1 / (p_CH1 + r);
    z_CH1 = z_CH1 + g * (input - z_CH1);
    p_CH1 = (1 - g) * p_CH1;
    z = z_CH1;
    break;
  case 2:
    p_CH2 = p_CH2 + q;
    g = p_CH2 / (p_CH2 + r);
    z_CH2 = z_CH2 + g * (input - z_CH2);
    p_CH2 = (1 - g) * p_CH2;
    z = z_CH2;
    break;
  case 3:
    p_CH3 = p_CH3 + q;
    g = p_CH3 / (p_CH3 + r);
    z_CH3 = z_CH3 + g * (input - z_CH3);
    p_CH3 = (1 - g) * p_CH3;
    z = z_CH3;
    break;
  default:
    break;
  }
  return z;
}


/**
 * @description: 低通滤波的使用方法
 * @return {*}
 */
// // 全局或静态变量，用于保存上一次的滤波结果
// static float filtered_value = 0.0f;
// // 滤波系数，根据实际需求调整
// // 常用范围: 0.01 - 0.3 (时间常数大，平滑好) 或 0.5 - 0.9 (响应快)
// #define ALPHA 0.2f
// void main_loop(void){
//     while(1) {
//         // 读取原始传感器数据
//         float raw_data = read_sensor();
//         // 应用低通滤波
//         filtered_value = low_pass_filter(raw_data, filtered_value, ALPHA);
//         // 使用滤波后的值
//         control_motor(filtered_value);
//         delay_ms(10); // 根据采样周期调整
//     }
// }

/**
 * @description: 一阶卡尔曼滤波的使用方式
 * @return {*}
 */
// // 定义噪声参数 - 需要根据实际系统调试
// #define MEASUREMENT_NOISE 10.0f    // R值，传感器噪声大则设大
// #define PROCESS_NOISE     0.001f   // Q值，系统变化快则设大
// void main_loop(void){
//     while(1) {
//         // 读取原始传感器数据
//         float raw_data = read_sensor();
//         // 应用卡尔曼滤波
//         float filtered_data = kalman_filter_std(raw_data, MEASUREMENT_NOISE, PROCESS_NOISE);
//         // 使用滤波后的值
//         control_motor(filtered_data);
//         delay_ms(10); // 根据采样周期调整
//     }
// }

