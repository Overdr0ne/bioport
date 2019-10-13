#ifndef __ANALOG_H_
#define __ANALOG_H_

#include <drivers/adc.h>
#include <hal/nrf_saadc.h>
#include <string.h>

#define ADC_DEVICE_NAME DT_ADC_0_NAME
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1
#define ADC_2ND_CHANNEL_ID 2
#define ADC_2ND_CHANNEL_INPUT NRF_SAADC_INPUT_AIN2

#define ANALOG_BUFFER_SIZE 6

struct device *init_adc(s16_t *m_sample_buffer);
void check_samples(int expected_count, s16_t *m_sample_buffer);

#endif // __ANALOG_H_
