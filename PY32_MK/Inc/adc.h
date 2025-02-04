/*
*   File: adc.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef ADC_H
#define ADC_H


#include <stdint.h>

void ADC_Init(void);
void ADC_Read(void);

float PY32_ADC_Temp_to_degC(uint32_t adc_temp);
float TMP235_V_to_degC(float vin);

#endif