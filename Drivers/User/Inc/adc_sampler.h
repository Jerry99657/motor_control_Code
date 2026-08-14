#ifndef ADC_SAMPLER_H
#define ADC_SAMPLER_H

#include "main.h"

void AdcSampler_Init(ADC_HandleTypeDef *adc);
void AdcSampler_Process(void);
void AdcSampler_RequestFromISR(void);
void AdcSampler_ConversionCompleteFromISR(ADC_HandleTypeDef *adc);

#endif /* ADC_SAMPLER_H */
