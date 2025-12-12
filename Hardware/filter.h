#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

float low_pass_filter(float input, float last_output, float alpha);
float kalman_filter_std(uint8_t Motor_CH, float input, float r, float q);


#endif // FILTER_H
