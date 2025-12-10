#ifndef __FILTER_H
#define __FILTER_H

float low_pass_filter(float input, float last_output, float alpha);
float kalman_filter_std(float input, float r, float q);
float kalman_filter_dir_on(float input, float r, float q);
float kalman_filter_dir_off(float input, float r, float q);


#endif // FILTER_H
