/*
*   File: adc.cpp
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#include <math.h>
#include "adc.h"
#include "app_data.h"
#include "alarm.h"
#include "error_handler.h"

#include "io/adc_init.h"
#include "io/gpio_init.h"

#include "py32f0xx_hal.h"

extern "C" {

#define VREFINT_V 1.20
#define THERMISTOR_NOMINAL	((float) 100000)	/* 100k */
#define TEMPERATURE_NOMINAL	((float) 25)		/* 25 degrees C */
#define B_CONSTANT			((float) 4250)	    /* B-constant (K) 25/50 degrees C */

#define BRIDGE_RESISTOR		((float) 100000)	/* 100k */
#define ADC_RESOLUTION		4095	            /* 12bit */


/**
 * @brief Converts ADC reading of internal reference voltage to MCU VCC voltage.
 * @param adc_val The raw ADC value from internal reference measurement.
 * @return The calculated VCC voltage in volts.
 */
float ADC_Vrefint_to_Vcc(uint32_t adc_val)
{
    float f;    
    f = (float) 4096.0 / (float) adc_val;
    return VREFINT_V * f;    
}

/**
 * @brief Converts raw ADC value to voltage based on VCC reference.
 * @param adc_val The raw ADC value to convert.
 * @param Vcc The reference VCC voltage.
 * @return The calculated voltage in volts.
 */
float ADC_to_Volts(uint32_t adc_val, float Vcc)
{
    float f;    
    f = (float) adc_val / (float) 4096.0;
    return Vcc * f;        
}

void ADC_Read(void)
{
/**
 * @brief Reads all configured ADC channels and updates application data.
 *
 * This function performs sequential ADC conversions for each relevant channel,
 * including sample temperature, valve temperature, battery sense, USB CC1/CC2 (if enabled),
 * MCU internal temperature sensor, and reference voltage. The results are stored in the
 * global data structure, and derived voltages and temperatures are calculated.
 * It also updates maximum temperature records and checks for overtemperature conditions,
 * triggering error handling if necessary.
 *
 * The function should be called periodically to keep sensor and system data up to date.
 * 
 * ADC values must be read in logical pin order (PA0, then PA1, etc)
 */
    float mcu_vcc;
            
    //Sample with ADC in polling mode
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[0] = HAL_ADC_GetValue(&AdcHandle); // Sample temperature
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[1] = HAL_ADC_GetValue(&AdcHandle); // Valve temperature

    if (data.usb_cc_adc_read_enabled) {
        HAL_ADC_Start(&AdcHandle);
        HAL_ADC_PollForConversion(&AdcHandle, 1000);
        data.adcReading[4] = HAL_ADC_GetValue(&AdcHandle); // USB_CC2               
    } 
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[2] = HAL_ADC_GetValue(&AdcHandle); // V_BATT_SENSE (system_input_voltage)
    
    if (data.usb_cc_adc_read_enabled) {
        HAL_ADC_Start(&AdcHandle);
        HAL_ADC_PollForConversion(&AdcHandle, 1000);
        data.adcReading[3] = HAL_ADC_GetValue(&AdcHandle); // USB_CC1        
    } 
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[5] = HAL_ADC_GetValue(&AdcHandle); // MCU Temp sensor
    
    HAL_ADC_Start(&AdcHandle);
    HAL_ADC_PollForConversion(&AdcHandle, 1000);
    data.adcReading[6] = HAL_ADC_GetValue(&AdcHandle); // MCU VrefInt

    mcu_vcc = ADC_Vrefint_to_Vcc(data.adcReading[6]);
    data.vcc_mcu_voltage = mcu_vcc;
    data.py32_temperature_c = PY32_ADC_Temp_to_degC(data.adcReading[5]);
    data.adcVoltage[0] = ADC_to_Volts (data.adcReading[0], mcu_vcc);
    data.adcVoltage[1] = ADC_to_Volts (data.adcReading[1], mcu_vcc);
    data.adcVoltage[2] = ADC_to_Volts (data.adcReading[2], mcu_vcc);

    if (data.usb_cc_adc_read_enabled) {
        data.adcVoltage[3] = ADC_to_Volts (data.adcReading[3], mcu_vcc);
        data.adcVoltage[4] = ADC_to_Volts (data.adcReading[4], mcu_vcc);
        data.usb_cc1_voltage = data.adcVoltage[3];
        data.usb_cc2_voltage = data.adcVoltage[4];
    } else {
        data.usb_cc1_voltage = -1.0;
        data.usb_cc2_voltage = -1.0;
    }
    
    data.sample_thermistor_v = data.adcVoltage[0];
    //data.sample_temperature_c = TMP235_V_to_degC(data.adcVoltage[0]);
    data.sample_thermistor_r = ADC_to_thermistor_resistance(data.adcReading[0]);
    data.sample_temperature_c = thermistor_r_to_temperature(data.sample_thermistor_r);
    
    data.valve_thermistor_v = data.adcVoltage[1];
    //data.valve_temperature_c = TMP235_V_to_degC(data.adcVoltage[1]);    
    data.valve_thermistor_r = ADC_to_thermistor_resistance(data.adcReading[1]);
    data.valve_temperature_c = thermistor_r_to_temperature(data.valve_thermistor_r);
    
    data.system_input_voltage = data.adcVoltage[2] * V_BATT_SENSE_MULTIPLIER;    
    
    if (data.sample_temperature_c > data.sample_max_temperature_c) {
        data.sample_max_temperature_c = data.sample_temperature_c;
    }
    
    if (data.valve_temperature_c > data.valve_max_temperature_c) {
        data.valve_max_temperature_c = data.valve_temperature_c;
    }
    
    if (data.sample_temperature_c >= OVERTEMP_ERR_C) {
		APP_ErrorHandler(ERR_OVERTEMP_SHUTDOWN);
    } else if (data.valve_temperature_c >= OVERTEMP_ERR_C) {
		APP_ErrorHandler(ERR_OVERTEMP_SHUTDOWN);
    }
		
    data.sh_pwm_during_adc_meas = data.sample_heater_pwm_value;    
    data.vh_pwm_during_adc_meas = data.valve_heater_pwm_value;    
    
}


// PY32 internal temperature conversion formula:
// temp_c = ((85c - 30c) / (HAL_ADC_TSCAL2 - HAL_ADC_TSCAL1)) * (ADC_TEMP - HAL_ADC_TSCAL1) - 30
//      The first part of this equation is pre-calculated and stored in: temperature_cal
/**
 * @brief Converts ADC temperature sensor reading to degrees Celsius.
 * @param adc_temp The raw ADC value from the MCU's internal temperature sensor.
 * @return The calculated temperature in degrees Celsius.
 * 
 * Uses the formula: temp_c = ((85°C - 30°C) / (TSCAL2 - TSCAL1)) * (ADC_TEMP - TSCAL1) + 30
 * where TSCAL1 and TSCAL2 are factory-calibrated values.
 */
float PY32_ADC_Temp_to_degC(uint32_t adc_temp)
{
    float temp_c;    
    temp_c = temperature_cal * ((float) adc_temp - (float) HAL_ADC_TSCAL1) + 30.0f;
    return temp_c;
}

/**
 * @brief Converts TMP235 sensor voltage to temperature.
 * @param vin The measured voltage from the TMP235 sensor.
 * @return The calculated temperature in degrees Celsius.
 * 
 * TMP235 sensor output is 10mV per degree C with a 500mV offset.
 * Formula: temperature = (voltage - 0.5V) / 0.01V/°C
 */
float TMP235_V_to_degC(float vin)
{
    float temp_c;
    temp_c = (vin - 0.5) / 0.01;
    return temp_c;
}

/**
 * @brief Converts ADC reading to thermistor resistance value.
 * @param raw_adc The raw ADC value from the thermistor voltage divider.
 * @return The calculated thermistor resistance in ohms.
 * 
 * Calculates thermistor resistance using voltage divider equation:
 * R_thermistor = ((ADC_MAX - ADC_value) * R_bridge) / ADC_value
 * where R_bridge is the fixed bridge resistor value.
 */
uint32_t ADC_to_thermistor_resistance(uint32_t raw_adc)
{
	float r;					
	r = ((ADC_RESOLUTION - raw_adc) * BRIDGE_RESISTOR) / (raw_adc);
	return (uint32_t) r;				
}    

/**
 * @brief Converts thermistor resistance to temperature using B-equation.
 * @param resistance The thermistor resistance in ohms.
 * @return The calculated temperature in degrees Celsius.
 * 
 * Uses the B-equation for NTC thermistors:
 * 1/T = 1/T0 + (1/B) * ln(R/R0)
 * where:
 * - T0 is nominal temperature (25°C)
 * - B is the B-constant
 * - R0 is nominal resistance at T0
 * Returns temperature rounded to 1 decimal place.
 */
float thermistor_r_to_temperature(uint32_t resistance)
{
	float temp;

	temp = (float) resistance / (float) THERMISTOR_NOMINAL;	// (R/Ro)
	temp = logf(temp);			// ln(R/Ro)
	temp /= B_CONSTANT;			// 1/B * ln(R/Ro)
	temp += 1.0 / (TEMPERATURE_NOMINAL + 273.15);	// + (1/To)
	temp = 1.0 / temp;			// Invert
	temp -= 273.15;				// convert to C
    
    // round the temperature to 1 decimal place:
    temp = roundf(temp * 10.0f) / 10.0f;

	return temp;
}

} // extern "C"