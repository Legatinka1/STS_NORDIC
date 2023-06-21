#ifndef GL_ADC_H__
#define GL_ADC_H__

#include "nrf_saadc.h"

#ifdef __cplusplus
extern "C" {
#endif


#define GL_ADC_PIN_NONE ((uint8_t)NRF_SAADC_INPUT_DISABLED)
#define GL_ADC_PIN_0_02 ((uint8_t)NRF_SAADC_INPUT_AIN0)
#define GL_ADC_PIN_0_03 ((uint8_t)NRF_SAADC_INPUT_AIN1)
#define GL_ADC_PIN_0_04 ((uint8_t)NRF_SAADC_INPUT_AIN2)
#define GL_ADC_PIN_0_05 ((uint8_t)NRF_SAADC_INPUT_AIN3)
#define GL_ADC_PIN_0_28 ((uint8_t)NRF_SAADC_INPUT_AIN4)
#define GL_ADC_PIN_0_29 ((uint8_t)NRF_SAADC_INPUT_AIN5)
#define GL_ADC_PIN_0_30 ((uint8_t)NRF_SAADC_INPUT_AIN6)
#define GL_ADC_PIN_0_31 ((uint8_t)NRF_SAADC_INPUT_AIN7)
#define GL_ADC_PIN_VDD  ((uint8_t)NRF_SAADC_INPUT_VDD)
#if defined(SAADC_CH_PSELP_PSELP_VDDHDIV5)
#define GL_ADC_PIN_VDDH ((uint8_t)NRF_SAADC_INPUT_VDDHDIV5)
#endif

#define GL_ADC_GAIN_1_6 SAADC_CH_CONFIG_GAIN_Gain1_6 // 1/6
#define GL_ADC_GAIN_1_5 SAADC_CH_CONFIG_GAIN_Gain1_5 // 1/5
#define GL_ADC_GAIN_1_4 SAADC_CH_CONFIG_GAIN_Gain1_4 // 1/4
#define GL_ADC_GAIN_1_3 SAADC_CH_CONFIG_GAIN_Gain1_3 // 1/3
#define GL_ADC_GAIN_1_2 SAADC_CH_CONFIG_GAIN_Gain1_2 // 1/2
#define GL_ADC_GAIN_1 SAADC_CH_CONFIG_GAIN_Gain1 // 1
#define GL_ADC_GAIN_2 SAADC_CH_CONFIG_GAIN_Gain2 // 2
#define GL_ADC_GAIN_4 SAADC_CH_CONFIG_GAIN_Gain4 // 4

int16_t gl_adc_read(uint8_t input);
void gl_adc_read_multiple(uint8_t *inputs, uint8_t count, int16_t *results);
int16_t gl_adc_read_differential(uint8_t inputp, uint8_t inputn, uint8_t gain);

#define GL_ADC_FREQUENCY_MIN 7816 // 16000000 / 2047
#define GL_ADC_FREQUENCY_MAX 200000 // 16000000 / 80

int gl_adc_continuous_buffer_start(uint8_t input, int16_t *buffer, uint32_t len, uint32_t chunks, uint32_t frequency, uint8_t gain, int (*callback)(int16_t *, uint32_t)); // Callback returs 1 to continue or 0 to stop
int gl_adc_continuous_buffer_is_running(void);
void gl_adc_continuous_buffer_stop(void);

void gl_adc_disable(void);

#define GL_ADC_VALUE_MAX 4095

#define GL_ADC_VALUE_TO_MILLIVOLTS(value) ((int32_t)((((int32_t)(value) * 225L) + 128L) >> 8))
#define GL_ADC_VALUE_TO_CENTIIVOLTS(value) ((int32_t)((((int32_t)(value) * 45L) + 256L) >> 9))
#define GL_ADC_VALUE_TO_DECIVOLTS(value) ((int32_t)((((int32_t)(value) * 9) + 512L) >> 10))

#define GL_ADC_VALUE_DIVIDE(value, r_up, r_down) ((int32_t)(((value) == 0) ? 0 : (((r_up) && (r_down)) ? (((r_up) == (r_down)) ? ((value) << 1) : ((((int32_t)(value) * ((r_up) + (r_down))) + ((r_down) >> 1)) / (r_down))) : (value))))

unsigned char adc_convert_value_to_battery_level(long value, int is_battery_rechargeable);
unsigned char adc_convert_millivolts_to_battery_level(long millivolts, int is_battery_rechargeable);
unsigned char adc_convert_centivolts_to_battery_level(long centivolts, int is_battery_rechargeable);
unsigned char adc_convert_decivolts_to_battery_level(long decivolts, int is_battery_rechargeable);


int32_t gl_temperature_read(void);

#define  GL_TEMPERATURE_TO_CELSIUS_DEGREES(value) ((int32_t)(value >> 2))
#define  GL_TEMPERATURE_TO_CELSIUS_DECIDEGREES(value) ((int32_t)((value * 5) >> 1))
#define  GL_TEMPERATURE_TO_CELSIUS_CENTIDEGREES(value) ((int32_t)(value * 25))


#ifdef __cplusplus
}
#endif

#endif // GL_ADC_H__
