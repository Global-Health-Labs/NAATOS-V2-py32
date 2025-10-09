#ifndef ADC_INIT_H
#define ADC_INIT_H

#include "main.h"
#include "alarm.h"

ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConf;

float temperature_cal;
/**
 * @brief Initializes ADC peripheral for the project.
 */
void ADC_Init(void);

#endif // ADC_INIT_H
