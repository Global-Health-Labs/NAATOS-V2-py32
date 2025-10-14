#ifndef ADC_INIT_H
#define ADC_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "alarm.h"

extern ADC_HandleTypeDef AdcHandle;
extern ADC_ChannelConfTypeDef AdcChanConf;

extern float temperature_cal;


void ADC_Set_USB_cc_read_state(bool enable_usb_cc_adc_read);

/**
 * @brief Initializes ADC peripheral for the project.
 */
void ADC_Init(void);

#ifdef __cplusplus
}
#endif

#endif // ADC_INIT_H
