#include "GL_I2S.h"
#include <stdint.h>

/* I2S */
#ifndef GL_I2S_PIN_DISCONNECTED
#define GL_I2S_PIN_DISCONNECTED 0xFF
#endif
#include <nrf.h>
static int st_i2s_initialized = 0;
static unsigned long st_i2s_sample_duration_us = 0;
void gl_i2s_init(uint8_t mck_pin, uint8_t sck_pin, uint8_t lrck_pin, uint8_t sdout_pin)
{
    if (st_i2s_initialized)
    {
        return;
    }
    else
    {
#ifdef NRF_I2S
        st_i2s_initialized = 1;
        NRF_GPIO->DIRSET = (1 << mck_pin) | (1 << sck_pin) | (1 << lrck_pin) | (1 << sdout_pin);
        NRF_I2S->CONFIG.RXEN = I2S_CONFIG_RXEN_RXEN_Disabled;
        NRF_I2S->CONFIG.TXEN = I2S_CONFIG_TXEN_TXEN_Enabled;
        NRF_I2S->CONFIG.MCKEN = I2S_CONFIG_MCKEN_MCKEN_Enabled;
        NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV21; // 32 MHz / 21 = 1.5238095
        NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_96X; // LRCK = MCK / 96 = 0.015873 (the closest one to 16 kHz that is possible to achieve)
        NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
        NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left;
        NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S;
        NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;
        NRF_I2S->PSEL.MCK = mck_pin;
        NRF_I2S->PSEL.SCK = sck_pin;
        NRF_I2S->PSEL.LRCK = lrck_pin;
        NRF_I2S->PSEL.SDIN = 255; // disconnect
        NRF_I2S->PSEL.SDOUT = sdout_pin;
        NRF_I2S->INTEN = I2S_INTEN_TXPTRUPD_Msk;
        NVIC_SetPriority(I2S_IRQn, 6);
        NVIC_ClearPendingIRQ(I2S_IRQn);
        NVIC_EnableIRQ(I2S_IRQn);
        st_i2s_sample_duration_us = 63; // the closest to 62.5 that is 16 kHz
#endif
    }
}
#ifdef NRF_I2S
static void (*st_i2s_irqhandler_callback)(void) = 0;
static int16_t st_i2s_pwm_table_high[2] = { 32767, 32767 };
static int16_t st_i2s_pwm_table_low[2] = { 0, 0 };
static uint32_t st_i2s_pwm_table_high_ptr = (uint32_t)&st_i2s_pwm_table_high[0];
static uint32_t st_i2s_pwm_table_low_ptr = (uint32_t)&st_i2s_pwm_table_low[0];
static volatile unsigned long st_i2s_pwm_countdown = 0;
static volatile unsigned long st_i2s_pwm_period = 0;
static volatile unsigned long st_i2s_pwm_duty_cyle = 0;
static volatile unsigned long st_i2s_pwm_cycle_index = 0;
static void st_i2s_irqhandler_callback_pwm(void)
{
    ++st_i2s_pwm_cycle_index;
    if (st_i2s_pwm_cycle_index == st_i2s_pwm_duty_cyle)
    {
        NRF_I2S->TXD.PTR = st_i2s_pwm_table_low_ptr;
    }
    else
    {
        if (st_i2s_pwm_cycle_index == st_i2s_pwm_period)
        {
            if (st_i2s_pwm_countdown <= 1)
            {
                NRF_I2S->ENABLE = 0;
                st_i2s_pwm_countdown = 0;
            }
            else
            {
                NRF_I2S->TXD.PTR = st_i2s_pwm_table_high_ptr;
                st_i2s_pwm_cycle_index = 0;
                --st_i2s_pwm_countdown;
            }
        }
    }
}
#endif
int gl_i2s_play_tone(unsigned long period_us, unsigned long duration_us)
{
#ifdef NRF_I2S
    unsigned long period, aligned_period_us, duty_cycle, count, half_sample_duration_us;
    if (!st_i2s_initialized)
        return 0;
    NRF_I2S->ENABLE = 0;
    if (!(period_us && duration_us))
        return 0;
    period = (period_us + (st_i2s_sample_duration_us >> 1)) / st_i2s_sample_duration_us;
    if (period < 2)
        period = 2;
    duty_cycle = period >> 1;
    aligned_period_us = period * st_i2s_sample_duration_us;
    if (duration_us < aligned_period_us)
        count = 1;
    else
        count = (duration_us + (aligned_period_us >> 1)) / aligned_period_us;
    st_i2s_irqhandler_callback = st_i2s_irqhandler_callback_pwm;
    NRF_I2S->ENABLE = 1;
    NRF_I2S->TXD.PTR = st_i2s_pwm_table_high_ptr;
    NRF_I2S->RXTXD.MAXCNT = 1;
    st_i2s_pwm_countdown = count;
    st_i2s_pwm_period = period;
    st_i2s_pwm_duty_cyle = duty_cycle;
    st_i2s_pwm_cycle_index = 0;
    NRF_I2S->TASKS_START = 1;
    return 1;
#else
    return 0;
#endif
}
#ifdef NRF_I2S
static short *st_i2s_buffer = 0;
#define I2S_BUFFER_CHUNK_SIZE 4
static short st_i2s_buffer_chunk_odd[I2S_BUFFER_CHUNK_SIZE * 2];
static short st_i2s_buffer_chunk_even[I2S_BUFFER_CHUNK_SIZE * 2];
static uint32_t st_i2s_buffer_chunk_odd_ptr = (uint32_t)&st_i2s_buffer_chunk_odd[0];
static uint32_t st_i2s_buffer_chunk_even_ptr = (uint32_t)&st_i2s_buffer_chunk_even[0];
static int st_i2s_buffer_chunk_is_odd = 0;
static unsigned long st_i2s_buffer_samples_count = 0;
static unsigned long st_i2s_buffer_samples_index = 0;
static int st_i2s_buffer_is_mono = 0;
static int st_i2s_buffer_do_loop = 0;
static void st_i2s_irqhandler_callback_buffer(void)
{
    unsigned long count, ptr, i;
    if (st_i2s_buffer_samples_index >= st_i2s_buffer_samples_count)
    {
        st_i2s_buffer_samples_index = 0;
        if (!st_i2s_buffer_do_loop)
        {
            st_i2s_buffer_chunk_is_odd = 0;
            NRF_I2S->ENABLE = 0;
            return;
        }
    }
    if ((st_i2s_buffer_samples_index + I2S_BUFFER_CHUNK_SIZE) >= st_i2s_buffer_samples_count)
        count = st_i2s_buffer_samples_count - st_i2s_buffer_samples_index;
    else
        count = I2S_BUFFER_CHUNK_SIZE;
    if (st_i2s_buffer_chunk_is_odd)
    {
        ptr = st_i2s_buffer_chunk_odd_ptr;
        st_i2s_buffer_chunk_is_odd = 0;
    }
    else
    {
        ptr = st_i2s_buffer_chunk_even_ptr;
        st_i2s_buffer_chunk_is_odd = 1;
    }
    if (st_i2s_buffer_is_mono)
    {
        unsigned short *src, *dest;
        src = ((unsigned short *)st_i2s_buffer) + st_i2s_buffer_samples_index;
        dest = (unsigned short *)ptr;
        i = count;
        while (i--)
        {
            *dest++ = *src;
            *dest++ = *src++;
        }
    }
    else
    {
        unsigned long *src, *dest;
        src = ((unsigned long *)st_i2s_buffer) + st_i2s_buffer_samples_index;
        dest = (unsigned long *)ptr;
        i = count;
        while (i--)
            *dest++ = *src++;
    }
    NRF_I2S->TXD.PTR = ptr;
    NRF_I2S->RXTXD.MAXCNT = count;
    st_i2s_buffer_samples_index += count;
}
#endif
int gl_i2s_play_buffer(const short *buf, unsigned long buf_samples_count, int is_mono, int do_loop)
{
#ifdef NRF_I2S
    unsigned long period, aligned_period_us, duty_cycle, count, half_sample_duration_us;
    if (!st_i2s_initialized)
        return 0;
    NRF_I2S->ENABLE = 0;
    if (!(buf && buf_samples_count))
        return 0;
    st_i2s_buffer = (short *)buf;
    st_i2s_buffer_samples_count = buf_samples_count;
    st_i2s_buffer_samples_index = 0;
    st_i2s_buffer_is_mono = is_mono;
    st_i2s_buffer_do_loop = do_loop;
    st_i2s_irqhandler_callback = st_i2s_irqhandler_callback_buffer;
    NRF_I2S->ENABLE = 1;
    st_i2s_irqhandler_callback_buffer();
    //NRF_I2S->TXD.PTR = st_i2s_buffer_chunk_even_ptr;
    //NRF_I2S->RXTXD.MAXCNT = I2S_BUFFER_CHUNK_SIZE;
    NRF_I2S->TASKS_START = 1;
    return 1;
#else
    return 0;
#endif
}
int gl_i2s_is_playing(void)
{
#ifdef NRF_I2S
    if (!st_i2s_initialized)
        return 0;
    return NRF_I2S->ENABLE ? 1 : 0;
#else
    return 0;
#endif
}
void gl_i2s_stop_playing(void)
{
#ifdef NRF_I2S
    if (st_i2s_initialized)
        NRF_I2S->ENABLE = 0;
#endif
}
#ifdef NRF_I2S
void I2S_IRQHandler()
{
    if (NRF_I2S->EVENTS_TXPTRUPD)
    {
        NRF_I2S->EVENTS_TXPTRUPD = 0; { volatile uint32_t dummy = NRF_I2S->EVENTS_TXPTRUPD; }
        if (st_i2s_irqhandler_callback)
            st_i2s_irqhandler_callback();
    }
}
#endif
/**/
