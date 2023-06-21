#include "GL_RTC.h"
#include <stdint.h>
#include <stdbool.h>

/* RTC */
/*
#define GL_RTC_TICKS_PER_SECOND 32768
#define GL_RTC_TICKS_TO_SECONDS(ticks) ((uint64_t)((ticks) >> 15))
#define GL_RTC_TICKS_TO_MILLISECONDS100(ticks) ((uint64_t)(((ticks) * 5ULL) >> 14))
#define GL_RTC_TICKS_TO_MILLISECONDS10(ticks) ((uint64_t)(((ticks) * 25ULL) >> 13))
#define GL_RTC_TICKS_TO_MILLISECONDS(ticks) ((uint64_t)(((ticks) * 125ULL) >> 12))
#define GL_RTC_TICKS_TO_MICROSECONDS100(ticks) ((uint64_t)(((ticks) * 625ULL) >> 11))
#define GL_RTC_TICKS_TO_MICROSECONDS10(ticks) ((uint64_t)(((ticks) * 3125ULL) >> 10))
#define GL_RTC_TICKS_TO_MICROSECONDS(ticks) ((uint64_t)(((ticks) * 15625ULL) >> 9))
*/
#include <nrfx.h>
#if NRFX_CHECK(NRFX_RTC_ENABLED)
    #if NRFX_CHECK(NRFX_RTC2_ENABLED)
        #define GL_RTC_ENABLED
        #include "nrf_drv_clock.h"
        #include "nrf_drv_rtc.h"
    #endif
#endif
#ifndef GL_RTC_ENABLED
    #if NRF_MODULE_ENABLED(APP_TIMER)
        #define GL_RTC_USES_APP_TIMER
        #include "app_timer.h"
    #endif
#endif
#define GL_RTC_OVERFLOW 0x01000000
static uint64_t st_rtc_overflows = 0;
#ifdef GL_RTC_ENABLED
static const nrf_drv_rtc_t st_rtc = NRF_DRV_RTC_INSTANCE(2);
static void st_rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_OVERFLOW)
        st_rtc_overflows += GL_RTC_OVERFLOW;
}
#endif
#ifdef GL_RTC_USES_APP_TIMER
static const uint64_t st_rtc_multiplier = GL_RTC_TICKS_PER_SECOND / APP_TIMER_TICKS(1000);
APP_TIMER_DEF(st_rtc_app_timer_id);
static uint32_t st_rtc_app_timer_cnt_last = 0;
static uint32_t st_rtc_app_timer_cnt_get(void)
{
    uint32_t cnt = app_timer_cnt_get();
    if (cnt < st_rtc_app_timer_cnt_last)
    {
        st_rtc_app_timer_cnt_last = cnt;
        st_rtc_overflows += GL_RTC_OVERFLOW;
    }
    else
    {
        st_rtc_app_timer_cnt_last = cnt;
    }
    return cnt;
}
static void st_rtc_app_timer_handler(void *p_context)
{
    st_rtc_app_timer_cnt_get();
}
#endif
static bool st_rtc_is_configured = false;
uint32_t gl_rtc_config(void)
{
#ifdef GL_RTC_ENABLED
    if (st_rtc_is_configured)
    {
        return NRF_SUCCESS;
    }
    else
    {
        uint32_t ret;
        nrf_drv_rtc_config_t config ; // = NRF_DRV_RTC_DEFAULT_CONFIG;
        config.prescaler            = 0;
        config.interrupt_priority   = 7;
        config.reliable             = false;
        config.tick_latency         = 65;
        if (!nrf_drv_clock_init_check())
            nrf_drv_clock_init();
        nrf_drv_clock_lfclk_request(NULL);
        while (!nrf_clock_lf_is_running())
            ;
        ret = nrf_drv_rtc_init(&st_rtc, &config, st_rtc_handler);
        if (ret == NRF_SUCCESS)
        {
            nrf_drv_rtc_overflow_enable(&st_rtc, true);
            nrf_drv_rtc_enable(&st_rtc);
            st_rtc_is_configured = true;
        }
        else
        {
            nrf_drv_clock_lfclk_release();
        }        
        return ret;
    }
#else
    #ifdef GL_RTC_USES_APP_TIMER
    if (st_rtc_is_configured)
    {
        return NRF_SUCCESS;
    }
    else
    {
        uint32_t ret;
        ret = app_timer_create(&st_rtc_app_timer_id, APP_TIMER_MODE_REPEATED, st_rtc_app_timer_handler);
        if (ret == NRF_SUCCESS)
        {
            ret = app_timer_start(st_rtc_app_timer_id, GL_RTC_OVERFLOW >> 1, 0);
            if (ret == NRF_SUCCESS)
            {
                st_rtc_app_timer_cnt_get();
                st_rtc_is_configured = true;
            }
        }
        return ret;
    }
    #else
    return NRF_ERROR_INVALID_STATE;
    #endif
#endif
}
uint64_t gl_rtc_ticks(void)
{
#ifdef GL_RTC_ENABLED
    //return st_rtc_overflows | ((uint64_t)nrf_drv_rtc_counter_get(&st_rtc));
    return st_rtc_overflows | st_rtc.p_reg->COUNTER;
#else
    #ifdef GL_RTC_USES_APP_TIMER
    return (st_rtc_overflows | st_rtc_app_timer_cnt_get()) * st_rtc_multiplier;
    #else
    return st_rtc_overflows;
    #endif
#endif
}
uint64_t gl_rtc_seconds(void)
{
    return GL_RTC_TICKS_TO_SECONDS(gl_rtc_ticks());
}
uint64_t gl_rtc_milliseconds(void)
{
    return GL_RTC_TICKS_TO_MILLISECONDS(gl_rtc_ticks());
}
uint64_t gl_rtc_microseconds(void)
{
    return GL_RTC_TICKS_TO_MICROSECONDS(gl_rtc_ticks());
}
static int64_t st_rtc_virtual_ticks_offset = 0;
void gl_rtc_set_virtual_ticks(uint64_t virtual_ticks)
{
    st_rtc_virtual_ticks_offset = virtual_ticks - gl_rtc_ticks();
}
int64_t gl_rtc_get_virtual_ticks_offset(void)
{
    return st_rtc_virtual_ticks_offset;
}
void gl_rtc_set_virtual_ticks_of_ticks(uint64_t virtual_ticks, uint64_t ticks)
{
    st_rtc_virtual_ticks_offset = virtual_ticks - ticks;
}
uint64_t gl_rtc_get_virtual_ticks_of_ticks(uint64_t ticks)
{
    return ticks + st_rtc_virtual_ticks_offset;
}
uint64_t gl_rtc_virtual_ticks(void)
{
    return gl_rtc_ticks() + st_rtc_virtual_ticks_offset;
}
uint64_t gl_rtc_virtual_seconds(void)
{
    return GL_RTC_TICKS_TO_SECONDS(gl_rtc_virtual_ticks());
}
uint64_t gl_rtc_virtual_milliseconds(void)
{
    return GL_RTC_TICKS_TO_MILLISECONDS(gl_rtc_virtual_ticks());
}
uint64_t gl_rtc_virtual_microseconds(void)
{
    return GL_RTC_TICKS_TO_MICROSECONDS(gl_rtc_virtual_ticks());
}
/**/

