/*
*   File: adc.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include "adc.h"
#include "app_data.h"

#define VREFINT_V 1.20

ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConf;

uint32_t adcReading[5] = {0, 0, 0, 0, 0};
char adcReadingStr[5];
float temperature_cal;

void ADC_Init(void)
{
	
	//Enable Clocks
	__HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
	__HAL_RCC_ADC_CLK_ENABLE();
	
	//Start calibration
	AdcHandle.Instance = ADC1;
	
	if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
		{
			APP_ErrorHandler();
		}
	
	//Populate ADC init data
	AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_HSI_DIV8; //HSI = 24 MHz (ADC Clock = 24 MHz / 8)
	AdcHandle.Init.Resolution = ADC_RESOLUTION_12B; //12 bits
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT; //Right aligned
	AdcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD; //Don't plan to use
	AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //single conversion
	AdcHandle.Init.LowPowerAutoWait = DISABLE; //use all the power
	AdcHandle.Init.ContinuousConvMode = DISABLE; //don't need for polling
	AdcHandle.Init.DiscontinuousConvMode = ENABLE; 
	AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START; //Will start ADC in code
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; //not going to use external trigger
	AdcHandle.Init.DMAContinuousRequests = DISABLE; //Not using DMA
	AdcHandle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN; //shouldn't matter for polling
	AdcHandle.Init.SamplingTimeCommon = ADC_SAMPLETIME_41CYCLES_5; //Setting conversion time to 41.5 cycles
		
	//Initialize ADC
	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		APP_ErrorHandler();
	}

    //ADC_CHANNEL_2             (PA2, pin 1 on PY32F003F1)
    //ADC_CHANNEL_3             (PA3, pin 2 on PY32F003F1)
    //ADC_CHANNEL_4             (PA4, pin 3 on PY32F003F1)
    //ADC_CHANNEL_TEMPSENSOR    (ch 11, 9 usec min sample time)
    //ADC_CHANNEL_VREFINT       (ch 12)
	
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_2; 
	AdcChanConf.Channel = ADC_CHANNEL_2;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
	AdcChanConf.Rank = ADC_CHANNEL_3; 
	AdcChanConf.Channel = ADC_CHANNEL_3;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
	AdcChanConf.Rank = ADC_CHANNEL_4; 
	AdcChanConf.Channel = ADC_CHANNEL_4;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_TEMPSENSOR; 
	AdcChanConf.Channel = ADC_CHANNEL_TEMPSENSOR;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
        
	// Set ADC rank and channel
	AdcChanConf.Rank = ADC_CHANNEL_VREFINT; 
	AdcChanConf.Channel = ADC_CHANNEL_VREFINT;     
	
	if (HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConf) != HAL_OK)
	{
		APP_ErrorHandler();
	}
    
    // precalculate part of the temperature calculation:
    temperature_cal = (float) (85 - 30) / (float) (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1);        
}


float ADC_Vrefint_to_Vcc(uint32_t adc_val)
{
    float f;    
    f = (float) 4096.0 / (float) adc_val;
    return VREFINT_V * f;    
}

float ADC_to_Volts(uint32_t adc_val, float Vcc)
{
    float f;    
    f = (float) adc_val / (float) 4096.0;
    return Vcc * f;        
}


void ADC_Read(void)
{
    float vcc, pa2_v, pa3_v, pa4_v;
    
    //Sample with ADC in polling mode
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[0] = HAL_ADC_GetValue(&AdcHandle); // PA2, test ADC input
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[1] = HAL_ADC_GetValue(&AdcHandle); // PA3, Sample temperature
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[2] = HAL_ADC_GetValue(&AdcHandle); // PA4, Valve temperature
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[3] = HAL_ADC_GetValue(&AdcHandle); // MCU Temp sensor
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    adcReading[4] = HAL_ADC_GetValue(&AdcHandle); // MCU VrefInt

    vcc = ADC_Vrefint_to_Vcc(adcReading[4]);
    data.battery_voltage = vcc;
    data.py32_temperature_c = PY32_ADC_Temp_to_degC(adcReading[3]);
    pa2_v = ADC_to_Volts (adcReading[0], vcc);
    pa3_v = ADC_to_Volts (adcReading[1], vcc);
    pa4_v = ADC_to_Volts (adcReading[2], vcc);
    
    data.test_adc_voltage = pa2_v;    
    data.sample_temperature_c = TMP235_V_to_degC(pa3_v);
    data.valve_temperature_c = TMP235_V_to_degC(pa4_v);    
}


// PY32 internal temperature conversion formula:
// temp_c = ((85c - 30c) / (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1)) * (ADC_TEMP - HAL_ADC_TSCAL1) - 30
//      The first part of this equation is pre-calculated and stored in: temperature_cal
float PY32_ADC_Temp_to_degC(uint32_t adc_temp)
{
    float temp_c;    
    temp_c = temperature_cal * ((float) adc_temp - (float) HAL_ADC_TSCAL1) + 30.0f;
    return temp_c;
}

// TMP235: temperature is 10 mv per deg_c with a 500 mv offset
//  (deg_c) = (v_in - 0.5) / .01
float TMP235_V_to_degC(float vin)
{
    float temp_c;
    temp_c = (vin - 0.5) / 0.01;
    return temp_c;
}

