#include "GL_ADC.h"
#include <stdint.h>

/* ADC */
#include "nrf_saadc.h"
/*
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
*/
#define ST_ADC_PIN_MIN  ((uint8_t)NRF_SAADC_INPUT_AIN0)
#if defined(SAADC_CH_PSELP_PSELP_VDDHDIV5)
#define ST_ADC_PIN_MAX  ((uint8_t)NRF_SAADC_INPUT_VDDHDIV5)
#else
#define ST_ADC_PIN_MAX  ((uint8_t)NRF_SAADC_INPUT_VDD)
#endif
#define ST_ADC_CONTINUOUS_FREQUENCY_CORE 16000000 // 16 MHz
#define ST_ADC_CONTINUOUS_FREQUENCY_CC_MIN 80 // 200000 Hz: 16000000 / 80
#define ST_ADC_CONTINUOUS_FREQUENCY_CC_MAX 2047 // 7816 Hz: 16000000 / 2047
#define ST_ADC_CONTINUOUS_MODE_IS_SAFE
static int st_saadc_initialized = 0;
static int st_saadc_irq_enabled = 0;
static int (*st_saadc_buffer_callback)(int16_t *, uint32_t) = 0;
static int16_t *st_saadc_buffer_data = 0;
static uint32_t st_saadc_buffer_chunk_len = 0;
static uint32_t st_saadc_buffer_chunk_count = 0;
static uint32_t st_saadc_buffer_chunk_index = 0;
static uint32_t st_saadc_buffer_pos = 0;
static int st_saadc_buffer_is_running = 0;
void SAADC_IRQHandler(void)
{
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    }
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
    }
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
        if (st_saadc_buffer_is_running/* && st_saadc_buffer_callback && st_saadc_buffer_data && st_saadc_buffer_chunk_len && st_saadc_buffer_chunk_count*/)
        {
            int16_t *buffer = st_saadc_buffer_data + st_saadc_buffer_pos;
            st_saadc_buffer_pos = st_saadc_buffer_pos ? 0 : st_saadc_buffer_chunk_len;
            ++st_saadc_buffer_chunk_index;
            if (st_saadc_buffer_chunk_index >= st_saadc_buffer_chunk_count)
            {
                st_saadc_buffer_chunk_index = 0;
                st_saadc_buffer_pos = 0;
            }
            else
            {
                st_saadc_buffer_pos = st_saadc_buffer_chunk_len * st_saadc_buffer_chunk_index;
            }
            nrf_saadc_buffer_init(st_saadc_buffer_data + st_saadc_buffer_pos, st_saadc_buffer_chunk_len);
#ifndef ST_ADC_CONTINUOUS_MODE_IS_SAFE
            if (st_saadc_buffer_chunk_count > 1)
                nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
#endif
            if (st_saadc_buffer_callback(buffer, st_saadc_buffer_chunk_len))
            {
#ifdef ST_ADC_CONTINUOUS_MODE_IS_SAFE
                nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
#else
                if (st_saadc_buffer_chunk_count <= 1)
                    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
#endif
            }
            else
            {
                st_saadc_buffer_is_running = 0;
                //nrf_saadc_disable();
            }
        }
    }
}
static void st_saadc_set_continuous_mode(uint32_t cc)
{
    if ((cc >= ST_ADC_CONTINUOUS_FREQUENCY_CC_MIN) && (cc <= ST_ADC_CONTINUOUS_FREQUENCY_CC_MAX))
        NRF_SAADC->SAMPLERATE =
            (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos) |
            (cc << SAADC_SAMPLERATE_CC_Pos);
    else
        NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;
}
static void st_saadc_init(void)
{
    nrf_saadc_channel_config_t config;
    uint8_t channel;
    nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);
    nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_enable();
    config.acq_time = NRF_SAADC_ACQTIME_10US;
    config.gain = NRF_SAADC_GAIN1_6;
    config.mode = NRF_SAADC_MODE_SINGLE_ENDED;
    config.pin_p = NRF_SAADC_INPUT_DISABLED;
    config.pin_n = NRF_SAADC_INPUT_DISABLED;
    config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;
    config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;
    config.burst = NRF_SAADC_BURST_DISABLED;
    for (channel = 0; channel < NRF_SAADC_CHANNEL_COUNT; channel++)
    {
        nrf_saadc_channel_init(channel, &config);
        nrf_saadc_channel_limits_set(channel, -32768, 32767);
    }
    st_saadc_set_continuous_mode(0);
    st_saadc_initialized = 1;
}
static nrf_saadc_value_t st_saadc_results[NRF_SAADC_CHANNEL_COUNT];
static uint8_t st_saadc_channels_count = 0;
static void st_saadc_set_channels_count(uint8_t count)
{
    uint8_t channel;
    if (count == st_saadc_channels_count)
        return;
    for (channel = count; channel < st_saadc_channels_count; channel++)
        nrf_saadc_channel_input_set(channel, NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
    nrf_saadc_buffer_init(st_saadc_results, count);
    st_saadc_channels_count = count;
}
static void st_saadc_set_gain(uint8_t channel, nrf_saadc_gain_t gain)
{
    if ((gain < NRF_SAADC_GAIN1_6) || (gain > NRF_SAADC_GAIN4))
        return;
    NRF_SAADC->CH[channel].CONFIG =
        (NRF_SAADC->CH[channel].CONFIG & ~SAADC_CH_CONFIG_GAIN_Msk) |
        (gain << SAADC_CH_CONFIG_GAIN_Pos);
}
static void st_saadc_set_mode(uint8_t channel, nrf_saadc_gain_t mode)
{
    if ((mode != NRF_SAADC_MODE_SINGLE_ENDED) && (mode != NRF_SAADC_MODE_DIFFERENTIAL))
        return;
    NRF_SAADC->CH[channel].CONFIG =
        (NRF_SAADC->CH[channel].CONFIG & ~SAADC_CH_CONFIG_MODE_Msk) |
        (mode << SAADC_CH_CONFIG_MODE_Pos);
}
static void st_saadc_convert(void)
{
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    while (!nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED))
        ;//__NOP();
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
    while (!nrf_saadc_event_check(NRF_SAADC_EVENT_END))
        ;//__NOP();
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
    while (!nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED))
        ;//__NOP();
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
}
static void st_saadc_irq_enable(void)
{
    if (!st_saadc_irq_enabled)
    //if (!NVIC_GetEnableIRQ(SAADC_IRQn))
    {
        NVIC_SetPriority(SAADC_IRQn, 6);
        NVIC_EnableIRQ(SAADC_IRQn);
        st_saadc_irq_enabled = 1;
    }
}
static void st_saadc_irq_disable(void)
{
    if (st_saadc_irq_enabled)
    //if (NVIC_GetEnableIRQ(SAADC_IRQn))
    {
        nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
        NVIC_DisableIRQ(SAADC_IRQn);
        nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
        while (!nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED))
            ;//__NOP();
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
        st_saadc_set_continuous_mode(0);
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
        nrf_saadc_buffer_init(st_saadc_results, st_saadc_channels_count);
        st_saadc_set_gain(0, NRF_SAADC_GAIN1_6);
        st_saadc_buffer_is_running = 0;
        st_saadc_irq_enabled = 0;
    }
}
static void st_saadc_validate(void)
{
    if (!st_saadc_initialized)
    //if (!nrf_saadc_enable_check())
        st_saadc_init();
    st_saadc_irq_disable();
}
int16_t gl_adc_read(uint8_t input)
{
    if ((input < ST_ADC_PIN_MIN) || (input > ST_ADC_PIN_MAX))
        return 0;
    st_saadc_validate();
    nrf_saadc_channel_input_set(0, (nrf_saadc_input_t)input, NRF_SAADC_INPUT_DISABLED);
    st_saadc_set_channels_count(1);
    st_saadc_convert();
    return st_saadc_results[0];
}
void gl_adc_read_multiple(uint8_t *inputs, uint8_t count, int16_t *results)
{
    uint8_t i, channel = 0, channels_count = 0, input;
    if ((!count) || (!results))
        return;
    if (!inputs)
    {
        for (i = 0; i < count; i++)
            results[i] = 0;
        return;
    }
    st_saadc_validate();
    for (i = 0; i < count; i++)
    {
        input = inputs[i];
        if ((channels_count < NRF_SAADC_CHANNEL_COUNT) &&
            (input >= ST_ADC_PIN_MIN) && (input <= ST_ADC_PIN_MAX))
        {
            nrf_saadc_channel_input_set(channels_count, (nrf_saadc_input_t)input, NRF_SAADC_INPUT_DISABLED);
            ++channels_count;
            results[i] = 1;
        }
        else
        {
            results[i] = 0;
        }
    }
    if (!channels_count)
        return;
    st_saadc_set_channels_count(channels_count);
    st_saadc_convert();
    for (i = 0; i < count, channel < channels_count; i++)
        if (results[i] == 1)
            results[i] = st_saadc_results[channel++];
}
int16_t gl_adc_read_differential(uint8_t inputp, uint8_t inputn, uint8_t gain)
{
    if ((inputp < ST_ADC_PIN_MIN) || (inputp > ST_ADC_PIN_MAX) || (inputn < ST_ADC_PIN_MIN) || (inputn > ST_ADC_PIN_MAX))
        return 0;
    st_saadc_validate();
    st_saadc_set_gain(0, gain);
    st_saadc_set_mode(0, NRF_SAADC_MODE_DIFFERENTIAL);
    nrf_saadc_channel_input_set(0, (nrf_saadc_input_t)inputp, (nrf_saadc_input_t)inputn);
    st_saadc_set_channels_count(1);
    st_saadc_convert();
    st_saadc_set_gain(0, NRF_SAADC_GAIN1_6);
    st_saadc_set_mode(0, NRF_SAADC_MODE_SINGLE_ENDED);
    nrf_saadc_channel_input_set(0, (nrf_saadc_input_t)inputp, NRF_SAADC_INPUT_DISABLED);
    return st_saadc_results[0];
}
int gl_adc_continuous_buffer_start(uint8_t input, int16_t *buffer, uint32_t len, uint32_t chunks, uint32_t frequency, uint8_t gain, int (*callback)(int16_t *, uint32_t)) // Callback returs 1 to continue or 0 to stop
{
    uint32_t cc;
    if ((input < ST_ADC_PIN_MIN) || (input > ST_ADC_PIN_MAX))
        return 0;
    if ((!buffer) || (len < chunks) || (!chunks) || (!frequency) || (!callback))
        return 0;
    cc = (ST_ADC_CONTINUOUS_FREQUENCY_CORE + (frequency >> 1)) / frequency;
    if ((cc < ST_ADC_CONTINUOUS_FREQUENCY_CC_MIN) || (cc > ST_ADC_CONTINUOUS_FREQUENCY_CC_MAX))
        return 0;
    st_saadc_buffer_callback = callback;
    st_saadc_validate();
    st_saadc_buffer_is_running = 1;
    st_saadc_irq_enable();
    st_saadc_set_gain(0, gain);
    nrf_saadc_channel_input_set(0, (nrf_saadc_input_t)input, NRF_SAADC_INPUT_DISABLED);
    st_saadc_set_channels_count(1);
    st_saadc_buffer_data = buffer;
    st_saadc_buffer_chunk_len = len / chunks;
    st_saadc_buffer_chunk_count = chunks;
    st_saadc_buffer_chunk_index = 0;
    st_saadc_buffer_pos = 0;
    nrf_saadc_buffer_init(st_saadc_buffer_data, st_saadc_buffer_chunk_len);
    st_saadc_set_continuous_mode(cc);
    nrf_saadc_int_enable(NRF_SAADC_INT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
    return 1;
}
int gl_adc_continuous_buffer_is_running(void)
{
    return st_saadc_buffer_is_running; // st_saadc_irq_enabled
}
void gl_adc_continuous_buffer_stop(void)
{
    st_saadc_validate();
}
void gl_adc_disable(void)
{
    if (!st_saadc_initialized)
    //if (!nrf_saadc_enable_check())
        return;
    st_saadc_irq_disable();
    nrf_saadc_disable();
    st_saadc_set_channels_count(0);
    st_saadc_initialized = 0;
}
/*
#define GL_ADC_VALUE_MAX 4095
#define GL_ADC_VALUE_TO_MILLIVOLTS(value) ((int32_t)((((int32_t)(value) * 225L) + 128L) >> 8))
#define GL_ADC_VALUE_TO_CENTIIVOLTS(value) ((int32_t)((((int32_t)(value) * 45L) + 256L) >> 9))
#define GL_ADC_VALUE_TO_DECIVOLTS(value) ((int32_t)((((int32_t)(value) * 9) + 512L) >> 10))
#define GL_ADC_VALUE_DIVIDE(value, r_up, r_down) ((int32_t)(((value) == 0) ? 0 : (((r_up) && (r_down)) ? (((r_up) == (r_down)) ? ((value) << 1) : ((((int32_t)(value) * ((r_up) + (r_down))) + ((r_down) >> 1)) / (r_down))) : (value))))
*/
static unsigned char st_adc_battery_percent[8] = { 15, 30, 45, 60, 70, 80, 90, 95 };
unsigned char adc_convert_decivolts_to_battery_level(long decivolts, int is_battery_rechargeable)
{
    unsigned char min_decivolts = is_battery_rechargeable ? 34 : 22;
    if (decivolts < min_decivolts)
        return 0;
    else
        if (decivolts >= (min_decivolts + 8))
            return 100;
        else
            return st_adc_battery_percent[decivolts - min_decivolts];
}
unsigned char adc_convert_value_to_battery_level(long value, int is_battery_rechargeable)
{
    if (value <= 0)
        return 0;
    return adc_convert_decivolts_to_battery_level(GL_ADC_VALUE_TO_DECIVOLTS(value), is_battery_rechargeable);
}
unsigned char adc_convert_millivolts_to_battery_level(long millivolts, int is_battery_rechargeable)
{
    if (millivolts <= 0)
        return 0;
    return adc_convert_decivolts_to_battery_level((millivolts + 50) / 100, is_battery_rechargeable);
}
unsigned char adc_convert_centivolts_to_battery_level(long centivolts, int is_battery_rechargeable)
{
    if (centivolts <= 0)
        return 0;
    return adc_convert_decivolts_to_battery_level((centivolts + 5) / 10, is_battery_rechargeable);
}
/**/

/* Temperature */
#ifdef SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#else
#include "nrf_temp.h"
static int st_temperature_initialized = 0;
#endif
int32_t gl_temperature_read(void)
{
    int32_t temp;
#ifdef SOFTDEVICE_PRESENT
    sd_temp_get(&temp);
#else
    if (!st_temperature_initialized)
    {
        nrf_temp_init();
        st_temperature_initialized = 1;
    }
    NRF_TEMP->TASKS_START = 1;
    while (!NRF_TEMP->EVENTS_DATARDY)
        __NOP();
    NRF_TEMP->EVENTS_DATARDY = 0;
    temp = nrf_temp_read();
    NRF_TEMP->TASKS_STOP = 1;
#endif
    return temp;
}
/*
#define  GL_TEMPERATURE_TO_CELSIUS_DEGREES(value) ((int32_t)(value >> 2))
#define  GL_TEMPERATURE_TO_CELSIUS_DECIDEGREES(value) ((int32_t)((value * 5) >> 1))
#define  GL_TEMPERATURE_TO_CELSIUS_CENTIDEGREES(value) ((int32_t)(value * 25))
*/
/**/

