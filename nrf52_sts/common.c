#include "common.h"

/* Debug */

#ifndef USE_DEBUG_OUTPUT_NONE
#define USE_DEBUG_OUTPUT_NONE 0
#endif
#ifndef USE_DEBUG_OUTPUT_UART
#define USE_DEBUG_OUTPUT_UART 1
#endif
#ifndef USE_DEBUG_OUTPUT_TWI
#define USE_DEBUG_OUTPUT_TWI 2
#endif
#ifndef USE_DEBUG_OUTPUT_SWO
#define USE_DEBUG_OUTPUT_SWO 3
#endif
#ifndef USE_DEBUG_OUTPUT_USB
#define USE_DEBUG_OUTPUT_USB 4
#endif
#ifndef USE_DEBUG_OUTPUT_RTT
#define USE_DEBUG_OUTPUT_RTT 5
#endif
#ifndef USE_DEBUG_OUTPUT_UART1
#define USE_DEBUG_OUTPUT_UART1 6
#endif

#ifndef USE_DEBUG_OUTPUT_DEFAULT
#define USE_DEBUG_OUTPUT_DEFAULT USE_DEBUG_OUTPUT_RTT
#endif

int use_debug_output = USE_DEBUG_OUTPUT_DEFAULT;

//// UART

//#define DEBUG_PRINT_CAN_USE_APP_UART

#ifdef DEBUG_PRINT_CAN_USE_APP_UART
#include "app_uart.h"
#endif

static void st_debug_print_uart(const char *buf, int len)
{
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    if (gl_com_is_initialized())
    {
#endif
        gl_com_write(buf, len);
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    }
    else
    {
        uint32_t err_code;
        int i;
        uint8_t b;
        for (i = 0; i < len; i++)
        {
            b = (uint8_t)buf[i];
            do
            {
                err_code = app_uart_put(b);
            } while (err_code == NRF_ERROR_BUSY);
        }
    }
#endif
}

static int st_debug_getchar_uart(void)
{
    uint8_t c;
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    if (gl_com_is_initialized())
    {
#endif
        if (gl_com_read(&c, 1))
            return c;
        return -1;
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    }
    else
    {
        if (app_uart_get(&c))
            return -1;
        return c;
    }
#endif
}

static void st_debug_flush_uart(void)
{
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    if (gl_com_is_initialized())
#endif
        gl_com_flush();
#ifdef DEBUG_PRINT_CAN_USE_APP_UART
    else
        app_uart_flush();
#endif
}

//// UART1

static void st_debug_print_uart1(const char *buf, int len)
{
    gl_com1_write(buf, len);
}

static int st_debug_getchar_uart1(void)
{
    uint8_t c;
    if (gl_com1_read(&c, 1))
        return c;
    return -1;
}

static void st_debug_flush_uart1(void)
{
    gl_com1_flush();
}

//// TWI

static void st_debug_print_twi(const char *buf, int len)
{
    gl_twi_write(0x22, buf, len);
}

static int st_debug_getchar_twi(void)
{
    uint8_t c;
    if (gl_twi_read(0x22, &c, 1))
        return -1;
    return c;
}

static void st_debug_flush_twi(void)
{
    //uint8_t c;
    //while (!gl_twi_read(0x22, &c, 1))
    //    ;
    uint8_t c = 0x0E;
    gl_twi_write(0x00, &c, 1);
}

//// SWO

#include "core_cm4.h"

static void st_debug_print_swo(const char *buf, int len)
{
#ifdef ENABLE_SWO
    int i;
    for (i = 0; i < len; i++)
        ITM_SendChar(buf[i]);
#endif
}

static int st_debug_getchar_swo(void)
{
    //return ITM_ReceiveChar();
    return -1;
}

static void st_debug_flush_swo(void)
{
    //while (ITM_ReceiveChar() != -1)
    //    ;
}

//// USB

static void st_debug_print_usb(const char *buf, int len)
{
    gl_usb_write(buf, len, 0);
}

static int st_debug_getchar_usb(void)
{
    uint8_t c;
    if (gl_usb_read(&c, 1, 0))
        return c;
    return -1;
}

static void st_debug_flush_usb(void)
{
    gl_usb_flush();
}

//// RTT

#include "SEGGER_RTT.h"

static void st_debug_print_rtt(const char *buf, int len)
{
    SEGGER_RTT_Write(0, buf, len);
}

static int st_debug_getchar_rtt(void)
{
    return SEGGER_RTT_GetKey();
}

static void st_debug_flush_rtt(void)
{
    //
}

//// Common

static void st_debug_print(const char *buf, int len)
{
    switch (use_debug_output)
    {
        case USE_DEBUG_OUTPUT_UART:
            st_debug_print_uart(buf, len);
            break;
        case USE_DEBUG_OUTPUT_TWI:
            st_debug_print_twi(buf, len);
            break;
        case USE_DEBUG_OUTPUT_SWO:
            st_debug_print_swo(buf, len);
            break;
        case USE_DEBUG_OUTPUT_USB:
            st_debug_print_usb(buf, len);
            break;
        case USE_DEBUG_OUTPUT_RTT:
            st_debug_print_rtt(buf, len);
            break;
        case USE_DEBUG_OUTPUT_UART1:
            st_debug_print_uart1(buf, len);
            break;
        default:
            break;
    }
}

static int st_debug_getchar(void)
{
    switch (use_debug_output)
    {
        case USE_DEBUG_OUTPUT_UART:
            return st_debug_getchar_uart();
        case USE_DEBUG_OUTPUT_TWI:
            return st_debug_getchar_twi();
        case USE_DEBUG_OUTPUT_SWO:
            return st_debug_getchar_swo();
        case USE_DEBUG_OUTPUT_USB:
            return st_debug_getchar_usb();
        case USE_DEBUG_OUTPUT_RTT:
            return st_debug_getchar_rtt();
        case USE_DEBUG_OUTPUT_UART1:
            return st_debug_getchar_uart1();
        default:
            return -1;
    }
}

static void st_debug_flush(void)
{
    switch (use_debug_output)
    {
        case USE_DEBUG_OUTPUT_UART:
            st_debug_flush_uart();
            break;
        case USE_DEBUG_OUTPUT_TWI:
            st_debug_flush_twi();
            break;
        case USE_DEBUG_OUTPUT_SWO:
            st_debug_flush_swo();
            break;
        case USE_DEBUG_OUTPUT_USB:
            st_debug_flush_usb();
            break;
        case USE_DEBUG_OUTPUT_RTT:
            st_debug_flush_rtt();
            break;
        case USE_DEBUG_OUTPUT_UART1:
            st_debug_flush_uart1();
            break;
        default:
            break;
    }
}

void d_prin(const void *buf, int len)
{
    if (!use_debug_output)
        return;
    if (!buf)
        return;
    if (len <= 0)
        return;
    st_debug_print((const char *)buf, len);
}

#include <string.h>

void d_print(const char *s)
{
    if (!use_debug_output)
        return;
    if (!s)
        return;
    if (!(*s))
        return;
    st_debug_print(s, strlen(s));
}

#include <stdio.h>
#include <stdarg.h>

void d_printf(const char *fmt, ...)
{
    if(!use_debug_output)
        return;
    if(!fmt)
        return;
    {
        char buf[256];
        int len;
        va_list args;
        va_start(args, fmt);
        len = vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        st_debug_print(buf, len);
    }
}

int d_getchar(void)
{
    if(!use_debug_output)
        return -1;
    return st_debug_getchar();
}

void d_flush(void)
{
    st_debug_flush();
}

/* Timer */

#include "app_timer.h"

APP_TIMER_DEF(st_periodical_timer_id);

static const unsigned short st_periodical_timer_one_second_in_ticks = APP_TIMER_TICKS(1000);

static int st_periodical_timer_is_latched = 0;
static unsigned short st_periodical_timer_counter = 0;
static unsigned long st_periodical_timer_watchdog_timeout_ticks = 0;
static unsigned long st_periodical_timer_watchdog_timeout_ticks_remaining = 0;
static unsigned short st_periodical_timer_recent_period_in_ticks = 0;

static void st_periodical_timer_handler(void * p_context)
{
    st_periodical_timer_is_latched = 1;
    ++st_periodical_timer_counter;
    if (st_periodical_timer_watchdog_timeout_ticks)
    {
        unsigned short recent_period_in_ticks = st_periodical_timer_recent_period_in_ticks;
        if (st_periodical_timer_watchdog_timeout_ticks_remaining <= recent_period_in_ticks)
        {
            st_periodical_timer_watchdog_timeout_ticks_remaining = 0;
            app_error_handler_bare(3);
        }
        st_periodical_timer_watchdog_timeout_ticks_remaining -= recent_period_in_ticks;
    }
}

static int st_periodical_timer_initialized = 0;
static int st_periodical_timer_activated = 0;

static void st_periodical_timer_init(void)
{
    if (st_periodical_timer_initialized)
        return;
    //app_timer_init();
    app_timer_create(&st_periodical_timer_id, APP_TIMER_MODE_REPEATED, st_periodical_timer_handler);
    st_periodical_timer_initialized = 1;
}

static void st_periodical_timer_set_period_in_ticks(unsigned short period)
{
    unsigned int period_in_ticks;
    st_periodical_timer_init();
    if (period > st_periodical_timer_one_second_in_ticks)
        return;
    period_in_ticks = period;
    if (st_periodical_timer_activated)
    {
        if (period_in_ticks && (period_in_ticks == st_periodical_timer_recent_period_in_ticks))
            return;
        app_timer_stop(st_periodical_timer_id);
        st_periodical_timer_activated = 0;
    }
    st_periodical_timer_recent_period_in_ticks = period_in_ticks;
    if (!period_in_ticks)
        return;
    app_timer_start(st_periodical_timer_id, period_in_ticks, 0);
    st_periodical_timer_activated = 1;
}

static unsigned short st_periodical_timer_period = 0;
static unsigned short st_periodical_timer_frequency = 0;
static unsigned short st_periodical_timer_period_in_ticks = 0;

void periodical_timer_set_period(unsigned short period)
{
    unsigned short period_in_ticks;
    st_periodical_timer_period = period;
    if (period)
    {
        if (period >= 1000)
        {
            st_periodical_timer_period = 1000;
            st_periodical_timer_frequency = 1;
            period_in_ticks = st_periodical_timer_one_second_in_ticks;
        }
        else
        {
            st_periodical_timer_frequency = (1000 + (period >> 1)) / period;
            period_in_ticks = APP_TIMER_TICKS(period);
        }
    }
    else
    {
        st_periodical_timer_frequency = 0;
        period_in_ticks = 0;
    }
    st_periodical_timer_set_period_in_ticks(period_in_ticks);
    st_periodical_timer_period_in_ticks = st_periodical_timer_recent_period_in_ticks;
}

unsigned short periodical_timer_get_period(void)
{
    return st_periodical_timer_period;
}

void periodical_timer_set_frequency(unsigned short frequency)
{
    unsigned short period_in_ticks;
    st_periodical_timer_frequency = frequency;
    if (frequency)
    {
        if (frequency >= st_periodical_timer_one_second_in_ticks)
        {
            st_periodical_timer_frequency = st_periodical_timer_one_second_in_ticks;
            period_in_ticks = 1;
        }
        else
        {
            if (frequency >= 1000)
                st_periodical_timer_period = 1;
            else
                st_periodical_timer_period = (1000 + (frequency >> 1)) / frequency;
            period_in_ticks = (st_periodical_timer_one_second_in_ticks + (frequency >> 1)) / frequency;
        }
    }
    else
    {
        st_periodical_timer_period = 0;
        period_in_ticks = 0;
    }
    st_periodical_timer_set_period_in_ticks(period_in_ticks);
    st_periodical_timer_period_in_ticks = st_periodical_timer_recent_period_in_ticks;
}

unsigned short periodical_timer_get_frequency(void)
{
    return st_periodical_timer_frequency;
}

void periodical_timer_sleep(void)
{
    st_periodical_timer_set_period_in_ticks(st_periodical_timer_one_second_in_ticks);
}

void periodical_timer_wakeup(void)
{
    st_periodical_timer_set_period_in_ticks(st_periodical_timer_period_in_ticks);
}

int periodical_timer_is_sleeping(void)
{
    return st_periodical_timer_recent_period_in_ticks ? (st_periodical_timer_recent_period_in_ticks < st_periodical_timer_one_second_in_ticks ? 0 : 1) : 1;
}

unsigned short periodical_timer_get_counter(void)
{
    return st_periodical_timer_counter;
}

int periodical_timer_counter_was_changed(void)
{
    if (st_periodical_timer_is_latched)
    {
        st_periodical_timer_is_latched = 0;
        return 1;
    }
    return 0;
}

void periodical_timer_set_watchdog_timeout(unsigned long timeout)
{
    st_periodical_timer_watchdog_timeout_ticks = APP_TIMER_TICKS(timeout);
    st_periodical_timer_watchdog_timeout_ticks_remaining = st_periodical_timer_watchdog_timeout_ticks;
}

void periodical_timer_watchdog_feed(void)
{
    st_periodical_timer_watchdog_timeout_ticks_remaining = st_periodical_timer_watchdog_timeout_ticks;
}


/* Interrupt */

#include "nrf_drv_gpiote.h"

// Pay attention to GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS in sdk_config.h
// If need 4 buttons and all 3 interrupts below the number should be 7

static int st_lotohi_interrupt_enabled = 0;
static int st_lotohi_interrupt_latched = 0;
static unsigned char st_lotohi_interrupt_pin = 0xFF;
static void (*st_lotohi_interrupt_callback)(int) = 0;
static int st_lotohi_interrupt_id = 0;

static void st_lotohi_gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    st_lotohi_interrupt_latched = 1;
    if (st_lotohi_interrupt_callback)
        st_lotohi_interrupt_callback(st_lotohi_interrupt_id);
}

static void st_lotohi_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    if (pin >= NUMBER_OF_PINS)
        return;
    if (!nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_init();
    config.pull = pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL;
    st_lotohi_interrupt_pin = pin;
    st_lotohi_interrupt_callback = callback;
    st_lotohi_interrupt_id = id;
    st_lotohi_interrupt_enabled = 0;
    st_lotohi_interrupt_latched = 0;
    if (nrf_drv_gpiote_in_init(pin, &config, st_lotohi_gpiote_event_handler) != NRF_SUCCESS)
        return;
    nrf_drv_gpiote_in_event_enable(pin, true);
    st_lotohi_interrupt_enabled = 1;
}

static void st_lotohi_interrupt_disable(void)
{
    nrf_drv_gpiote_in_event_disable(st_lotohi_interrupt_pin);
    nrf_drv_gpiote_in_uninit(st_lotohi_interrupt_pin);
    st_lotohi_interrupt_enabled = 0;
    st_lotohi_interrupt_latched = 0;
}

int lotohi_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    if (st_lotohi_interrupt_enabled)
        st_lotohi_interrupt_disable();
    st_lotohi_interrupt_enable(pin, pull, id, callback);
    return st_lotohi_interrupt_enabled;
}

int lotohi_interrupt_is_enabled(void)
{
    return st_lotohi_interrupt_enabled;
}

int lotohi_interrupt_was_latched(void)
{
    int ret = st_lotohi_interrupt_latched;
    st_lotohi_interrupt_latched = 0;
    return ret;
}

void lotohi_interrupt_disable(void)
{
    if (st_lotohi_interrupt_enabled)
        st_lotohi_interrupt_disable();
}

static int st_hitolo_interrupt_enabled = 0;
static int st_hitolo_interrupt_latched = 0;
static unsigned char st_hitolo_interrupt_pin = 0xFF;
static void (*st_hitolo_interrupt_callback)(int) = 0;
static int st_hitolo_interrupt_id = 0;

static void st_hitolo_gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    st_hitolo_interrupt_latched = 1;
    if (st_hitolo_interrupt_callback)
        st_hitolo_interrupt_callback(st_hitolo_interrupt_id);
}

static void st_hitolo_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    if (pin >= NUMBER_OF_PINS)
        return;
    if (!nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_init();
    config.pull = pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL;
    st_hitolo_interrupt_pin = pin;
    st_hitolo_interrupt_callback = callback;
    st_hitolo_interrupt_id = id;
    st_hitolo_interrupt_enabled = 0;
    st_hitolo_interrupt_latched = 0;
    if (nrf_drv_gpiote_in_init(pin, &config, st_hitolo_gpiote_event_handler) != NRF_SUCCESS)
        return;
    nrf_drv_gpiote_in_event_enable(pin, true);
    st_hitolo_interrupt_enabled = 1;
}

static void st_hitolo_interrupt_disable(void)
{
    nrf_drv_gpiote_in_event_disable(st_hitolo_interrupt_pin);
    nrf_drv_gpiote_in_uninit(st_hitolo_interrupt_pin);
    st_hitolo_interrupt_enabled = 0;
    st_hitolo_interrupt_latched = 0;
}

int hitolo_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    if (st_hitolo_interrupt_enabled)
        st_hitolo_interrupt_disable();
    st_hitolo_interrupt_enable(pin, pull, id, callback);
    return st_hitolo_interrupt_enabled;
}

int hitolo_interrupt_is_enabled(void)
{
    return st_hitolo_interrupt_enabled;
}

int hitolo_interrupt_was_latched(void)
{
    int ret = st_hitolo_interrupt_latched;
    st_hitolo_interrupt_latched = 0;
    return ret;
}

void hitolo_interrupt_disable(void)
{
    if (st_hitolo_interrupt_enabled)
        st_hitolo_interrupt_disable();
}

static int st_toggle_interrupt_enabled = 0;
static int st_toggle_interrupt_latched = 0;
static unsigned char st_toggle_interrupt_pin = 0xFF;
static void (*st_toggle_interrupt_callback)(int) = 0;
static int st_toggle_interrupt_id = 0;

static void st_toggle_gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    st_toggle_interrupt_latched = 1;
    if (st_toggle_interrupt_callback)
        st_toggle_interrupt_callback(st_toggle_interrupt_id);
}

static void st_toggle_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    if (pin >= NUMBER_OF_PINS)
        return;
    if (!nrf_drv_gpiote_is_init())
        nrf_drv_gpiote_init();
    config.pull = pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL;
    st_toggle_interrupt_pin = pin;
    st_toggle_interrupt_callback = callback;
    st_toggle_interrupt_id = id;
    st_toggle_interrupt_enabled = 0;
    st_toggle_interrupt_latched = 0;
    if (nrf_drv_gpiote_in_init(pin, &config, st_toggle_gpiote_event_handler) != NRF_SUCCESS)
        return;
    nrf_drv_gpiote_in_event_enable(pin, true);
    st_toggle_interrupt_enabled = 1;
}

static void st_toggle_interrupt_disable(void)
{
    nrf_drv_gpiote_in_event_disable(st_toggle_interrupt_pin);
    nrf_drv_gpiote_in_uninit(st_toggle_interrupt_pin);
    st_toggle_interrupt_enabled = 0;
    st_toggle_interrupt_latched = 0;
}

int toggle_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int))
{
    if (st_toggle_interrupt_enabled)
        st_toggle_interrupt_disable();
    st_toggle_interrupt_enable(pin, pull, id, callback);
    return st_toggle_interrupt_enabled;
}

int toggle_interrupt_is_enabled(void)
{
    return st_toggle_interrupt_enabled;
}

int toggle_interrupt_was_latched(void)
{
    int ret = st_toggle_interrupt_latched;
    st_toggle_interrupt_latched = 0;
    return ret;
}

void toggle_interrupt_disable(void)
{
    if (st_toggle_interrupt_enabled)
        st_toggle_interrupt_disable();
}

/* Wakeup */

int lotohi_wakeup_enable(unsigned char pin, int pull)
{
    if (pin >= NUMBER_OF_PINS)
        return 0;
    nrf_gpio_cfg_sense_input(pin, pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
    return 1;
}

int hitolo_wakeup_enable(unsigned char pin, int pull)
{
    if (pin >= NUMBER_OF_PINS)
        return 0;
    nrf_gpio_cfg_sense_input(pin, pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    return 1;
}

int toggle_wakeup_enable(unsigned char pin, int pull)
{
    if (pin >= NUMBER_OF_PINS)
        return 0;
    nrf_gpio_cfg_input(pin, pull ? (pull > 0 ? NRF_GPIO_PIN_PULLUP : NRF_GPIO_PIN_PULLDOWN) : NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_sense_set(pin, nrf_gpio_pin_read(pin) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH);
    return 1;
}

/* Helper */

void swap_shorts(void *data, int count)
{
    char c, *p0 = (char *)data, *p1;
    int i;
    if ((!data) || (!count))
        return;
    for (i = 0; i < count; i++)
    {
        p1 = p0 + 1;
        c = *p0;
        *p0 = *p1;
        *p1 = c;
        p0 += 2;
    }
}

void swap_longs(void *data, int count)
{
    char c, *p0 = (char *)data, *p1, *p2, *p3;
    int i;
    if ((!data) || (!count))
        return;
    for (i = 0; i < count; i++)
    {
        p1 = p0 + 1;
        p2 = p0 + 2;
        p3 = p0 + 3;
        c = *p0;
        *p0 = *p3;
        *p3 = c;
        c = *p1;
        *p1 = *p2;
        *p2 = c;
        p0 += 4;
    }
}

unsigned char crcae(const void *data, unsigned int len)
{
    unsigned char crc = 0, *p = (unsigned char *)data;
    if (!data)
        return 0;
    while (len--)
        crc +=  (crc << 1) + (*p++);
    return crc;
}


