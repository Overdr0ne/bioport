#include "analog.h"

const struct adc_sequence sequence;

static const struct adc_channel_cfg m_1st_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_1ST_CHANNEL_INPUT,
#endif
};
#if defined(ADC_2ND_CHANNEL_ID)
static const struct adc_channel_cfg m_2nd_channel_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive = ADC_2ND_CHANNEL_INPUT,
#endif
};
#endif /* defined(ADC_2ND_CHANNEL_ID) */

struct device *init_adc(s16_t *m_sample_buffer) {
  int ret;
  struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

  adc_dev = device_get_binding(ADC_DEVICE_NAME);
  if (!adc_dev) {
    printk("failed to get adc dt binding\n");
  }

  ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
  if (ret != 0) {
    printk("Setting up of the first channel failed with code %d", ret);
  }

#if defined(ADC_2ND_CHANNEL_ID)
  ret = adc_channel_setup(adc_dev, &m_2nd_channel_cfg);
  if (ret != 0) {
    printk("Setting up of the second channel failed with code %d", ret);
  }
#endif /* defined(ADC_2ND_CHANNEL_ID) */

  (void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

  return adc_dev;
}

void adc_print_samples(int expected_count, s16_t *m_sample_buffer) {
  int i;

  printk("Samples read: ");
  for (i = 0; i < ANALOG_BUFFER_SIZE; i++) {
    s16_t sample_value = m_sample_buffer[i];

    printk("0x%04x ", sample_value);
    if (i < expected_count) {
      if (sample_value == 0) {
        printk("[%u] should be non-zero", i);
      }
    } else {
      if (sample_value != 0) {
        printk("[%u] should be zero", i);
      }
    }
  }
  printk("\n");
}
