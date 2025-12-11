#ifndef __NRF_PRINTF_H
#define __NRF_PRINTF_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct{
    float Debug_data0;
    float Debug_data1;
    float Debug_data2;
}NRF24_STRUCT;

extern float nRF_DataA,nRF_DataB,nRF_DataC;
void nRF_Printf(float *num, uint8_t count);
void nRF_Loop(void);

#endif // NRF_PRINTF_H
