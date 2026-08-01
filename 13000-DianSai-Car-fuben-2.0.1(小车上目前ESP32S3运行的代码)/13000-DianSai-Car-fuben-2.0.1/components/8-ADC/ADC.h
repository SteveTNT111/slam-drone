#ifndef __ADC_H_
#define __ADC_H_
#include <stdint.h>
#include "esp_adc/adc_oneshot.h"//单次转换头文件

extern uint16_t adc_ch0_datavalue;
extern uint16_t adc_ch1_datavalue;
void ADC_Init_Multiple_Conversions(void);

extern adc_oneshot_unit_handle_t Adc_Oneshot_Unit_Handle;
void ADC_Init_Standalone_Conversion(void);

#endif