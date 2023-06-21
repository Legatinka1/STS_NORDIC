#include "GL_PWM.h"
#include <stdint.h>

/* PWM */
#ifndef GL_PWM_PIN_DISCONNECTED
#define GL_PWM_PIN_DISCONNECTED 0xFF
#endif
#include <nrf.h>
#ifdef NRF_PWM3
#define ST_NRF_PWM_COUNT 4
NRF_PWM_Type* st_pwm_types[ST_NRF_PWM_COUNT] = { NRF_PWM0, NRF_PWM1, NRF_PWM2, NRF_PWM3 };
#else
#ifdef NRF_PWM2
#define ST_NRF_PWM_COUNT 3
NRF_PWM_Type* st_pwm_types[ST_NRF_PWM_COUNT] = { NRF_PWM0, NRF_PWM1, NRF_PWM2 };
#else
#ifdef NRF_PWM1
#define ST_NRF_PWM_COUNT 2
NRF_PWM_Type* st_pwm_types[ST_NRF_PWM_COUNT] = { NRF_PWM0, NRF_PWM1 };
#else
#ifdef NRF_PWM0
#define ST_NRF_PWM_COUNT 1
NRF_PWM_Type* st_pwm_types[ST_NRF_PWM_COUNT] = { NRF_PWM0 };
#else
#endif
#endif
#endif
#endif
int gl_pwm_is_playing(unsigned char drv)
{
    NRF_PWM_Type* PWMn;
    if (drv >= ST_NRF_PWM_COUNT)
        return 0;
    PWMn = st_pwm_types[drv];
    return PWMn->ENABLE ? (PWMn->EVENTS_STOPPED ? 0 : 1) : 0;
}
void gl_pwm_stop_playing(unsigned char drv)
{
    NRF_PWM_Type* PWMn;
    if (!gl_pwm_is_playing(drv))
        return;
    PWMn = st_pwm_types[drv];
    PWMn->TASKS_STOP = 1;
    while (PWMn->EVENTS_STOPPED)
        ;
    PWMn->ENABLE = 0;
}
void gl_pwm_set_pins(unsigned char drv, unsigned char pin0, unsigned char pin1, unsigned char pin2, unsigned char pin3)
{
    NRF_PWM_Type* PWMn;
    if (drv >= ST_NRF_PWM_COUNT)
        return;
    gl_pwm_stop_playing(drv);
#ifndef SOFTDEVICE_PRESENT
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) ;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
#endif
    NRF_GPIO->DIRSET = (1 << pin0) | (1 << pin1) | (1 << pin2) | (1 << pin3);
    NRF_GPIO->OUTCLR = (1 << pin0) | (1 << pin1) | (1 << pin2) | (1 << pin3);
    PWMn = st_pwm_types[drv];
    PWMn->PSEL.OUT[0] = pin0 < 0xFFU ? pin0 : 0xFFFFFFFF;
    PWMn->PSEL.OUT[1] = pin1 < 0xFFU ? pin1 : 0xFFFFFFFF;
    PWMn->PSEL.OUT[2] = pin2 < 0xFFU ? pin2 : 0xFFFFFFFF;
    PWMn->PSEL.OUT[3] = pin3 < 0xFFU ? pin3 : 0xFFFFFFFF;
}
static uint16_t st_pwm_table_single[4] = { 0x8001, 0x8001, 0x8001, 0x8001 };
int gl_pwm_play_tone(unsigned char drv, unsigned long period_us, unsigned long duration_us) // period_us: 1..16383 (0x0001..0x3FFF) - 1 MHz down to 61 Hz
{
    NRF_PWM_Type* PWMn;
    uint16_t top_value, duty_cycle;
    uint32_t count, ptr;
    if (drv >= ST_NRF_PWM_COUNT)
        return 0;
    gl_pwm_stop_playing(drv);
    if (!(period_us && duration_us))
        return 0;
    if (period_us >= 0x3FFFU)
        top_value = 0x7FFEU;
    else
        top_value = (uint16_t)(period_us << 1);
    duty_cycle = top_value >> 1;
    count = duration_us < period_us ? 1 : duration_us / period_us;
    st_pwm_table_single[drv] = 0x8000U | duty_cycle;
    ptr = (uint32_t)&st_pwm_table_single[drv];
    PWMn = st_pwm_types[drv];
    PWMn->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_8; // 2 us
    PWMn->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    PWMn->LOOP = (((count >> 1) + (count & 1) << PWM_SEQ_CNT_CNT_Pos));
    PWMn->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    PWMn->COUNTERTOP = top_value;
    PWMn->SEQ[0].CNT = 1; // ((sizeof(st_pwm_table_single[0]) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    PWMn->SEQ[0].ENDDELAY = 0;
    PWMn->SEQ[0].PTR = ptr;
    PWMn->SEQ[0].REFRESH = 0;
    PWMn->SEQ[1].CNT = 1;
    PWMn->SEQ[1].ENDDELAY = 0;
    PWMn->SEQ[1].PTR = ptr;
    PWMn->SEQ[1].REFRESH = 0;
    PWMn->SHORTS = PWM_SHORTS_LOOPSDONE_STOP_Msk;
    PWMn->EVENTS_STOPPED = 0; { volatile uint32_t dummy = PWMn->EVENTS_STOPPED; }
    PWMn->ENABLE = 1;
    PWMn->TASKS_SEQSTART[count & 1] = 1;
    return 1;
}
int gl_pwm_play_value(unsigned char drv, unsigned short period_us, unsigned short duty_cycle_us) // period_us: 1..32768 (0x0000..0x8000) - up to 15 bits, duty_cycle_us: 0..period_us-1
{
    NRF_PWM_Type* PWMn;
    uint16_t top_value, duty_cycle;
    uint32_t ptr;
    if (drv >= ST_NRF_PWM_COUNT)
        return 0;
    gl_pwm_stop_playing(drv);
    PWMn = st_pwm_types[drv];
    if (!duty_cycle_us)
    {
        NRF_GPIO->OUTCLR = (1 << PWMn->PSEL.OUT[0]) | (1 << PWMn->PSEL.OUT[1]) | (1 << PWMn->PSEL.OUT[2]) | (1 << PWMn->PSEL.OUT[3]);
        return 1;
    }
    if (period_us >= 0x7FFFU)
        top_value = 0x7FFFU;
    else
        if (period_us <= 1)
            top_value = 0;
        else
            top_value = (uint16_t)(period_us - 1);
    if (duty_cycle_us >= top_value)
    {
        NRF_GPIO->OUTSET = (1 << PWMn->PSEL.OUT[0]) | (1 << PWMn->PSEL.OUT[1]) | (1 << PWMn->PSEL.OUT[2]) | (1 << PWMn->PSEL.OUT[3]);
        return 1;
    }
    duty_cycle = (uint16_t)duty_cycle_us;
    st_pwm_table_single[drv] = 0x8000U | duty_cycle;
    ptr = (uint32_t)&st_pwm_table_single[drv];
    PWMn->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16; // 1 us
    PWMn->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    PWMn->LOOP = 0; // (PWM_LOOP_CNT_Disabled << PWM_SEQ_CNT_CNT_Pos));
    PWMn->DECODER = (PWM_DECODER_LOAD_Common << PWM_DECODER_LOAD_Pos) | (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    PWMn->COUNTERTOP = top_value;
    PWMn->SEQ[0].CNT = 1; // ((sizeof(st_pwm_table_single[0]) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    PWMn->SEQ[0].ENDDELAY = 0;
    PWMn->SEQ[0].PTR = ptr;
    PWMn->SEQ[0].REFRESH = 0;
    PWMn->SEQ[1].CNT = 0;
    PWMn->SEQ[1].ENDDELAY = 0;
    PWMn->SEQ[1].PTR = 0;
    PWMn->SEQ[1].REFRESH = 0;
    PWMn->SHORTS = 0;
    PWMn->EVENTS_STOPPED = 0; { volatile uint32_t dummy = PWMn->EVENTS_STOPPED; }
    PWMn->ENABLE = 1;
    PWMn->TASKS_SEQSTART[0] = 1;
    return 1;
}
/**/
