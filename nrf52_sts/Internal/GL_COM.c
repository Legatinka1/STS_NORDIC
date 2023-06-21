#include "GL_COM.h"

/* COM */
#ifndef GL_COM_PIN_DISCONNECTED
#define GL_COM_PIN_DISCONNECTED 0xFF
#endif
#include <nrfx.h>
#if UART0_ENABLED==1
#define INTERNAL_UART0_ENABLED
#define INTERNAL_UART_ENABLED
#endif
#if UART1_ENABLED==1
#define INTERNAL_UART1_ENABLED
#ifndef INTERNAL_UART_ENABLED
#define INTERNAL_UART_ENABLED
#endif
#endif
#ifdef INTERNAL_UART_ENABLED
#include "nrf_drv_uart.h"
#include "nrf_drv_clock.h"
#if APP_UART_ENABLED==1
#include "app_uart.h"
#endif
__STATIC_INLINE
ret_code_t fix_drv_uart_tx(nrf_drv_uart_t const * p_instance,
                           uint8_t const *        p_data,
                           uint32_t               length)
/*
{
    uint32_t result = 0;
    if (NRF_DRV_UART_USE_UARTE)
    {
        result = nrfx_uarte_tx(&p_instance->uarte,
                               p_data,
                               length);
    }
    else if (NRF_DRV_UART_USE_UART)
    {
        result = nrfx_uart_tx(&p_instance->uart,
                              p_data,
                              length);
    }
    return result;
}
*/
{
    if (length <= 255)
        return nrf_drv_uart_tx(p_instance, p_data, length);
    {
        uint32_t ret;
        uint32_t length_rest = length, length_chunk;
        uint8_t *p_data_pos = (uint8_t *)p_data;
        while (length_rest)
        {
            length_chunk = length_rest > 255 ? 255 : length_rest;
            ret = nrf_drv_uart_tx(p_instance, p_data_pos, length_chunk);
            if (ret != NRF_SUCCESS)
                return ret;
            length_rest -= length_chunk;
            p_data_pos += length_chunk;
        }
        return NRF_SUCCESS;
    }
}
static const uint32_t st_com_baudrates[] = {
    1200, NRF_UART_BAUDRATE_1200,
    2400, NRF_UART_BAUDRATE_2400,
    4800, NRF_UART_BAUDRATE_4800,
    9600, NRF_UART_BAUDRATE_9600,
    14400, NRF_UART_BAUDRATE_14400,
    19200, NRF_UART_BAUDRATE_19200,
    28800, NRF_UART_BAUDRATE_28800,
    31250, NRF_UART_BAUDRATE_31250,
    38400, NRF_UART_BAUDRATE_38400,
    56000, NRF_UART_BAUDRATE_56000,
    57600, NRF_UART_BAUDRATE_57600,
    76800, NRF_UART_BAUDRATE_76800,
    115200, NRF_UART_BAUDRATE_115200,
    230400, NRF_UART_BAUDRATE_230400,
    250000, NRF_UART_BAUDRATE_250000,
    460800, NRF_UART_BAUDRATE_460800,
    921600, NRF_UART_BAUDRATE_921600,
    1000000, NRF_UART_BAUDRATE_1000000,
    0, 0
};
static nrf_uart_baudrate_t st_com_get_baudrate(unsigned long baudrate)
{
    uint32_t *p = (uint32_t *)st_com_baudrates, ret = st_com_baudrates[1];
    while ((*p) && (baudrate >= *p))
    {
        ++p;
        ret = *p++;
    }
    return (nrf_uart_baudrate_t)ret;
}
#include "app_fifo.h"
#endif
/* UART 0 */
#ifdef INTERNAL_UART0_ENABLED
static const nrf_drv_uart_t st_com_instance = NRF_DRV_UART_INSTANCE(0);
static int st_com_initialized = 0;
static int st_com_tx_pin_present = 0;
static int st_com_rx_pin_present = 0;
static int st_com_uses_handler = 0;
static unsigned char st_com_buf_rx[512];
static app_fifo_t st_com_ringbuf_rx = { .p_buf = st_com_buf_rx, .buf_size_mask = sizeof(st_com_buf_rx) - 1, .read_pos = 0, .write_pos = 0 };
static unsigned char st_com_reqbuf_rx[1];
static void (*st_com_event_handler_custom_on_receive)(unsigned char) = 0;
static void st_com_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_RX_DONE:
            if (st_com_event_handler_custom_on_receive)
                st_com_event_handler_custom_on_receive(p_event->data.rxtx.p_data[0]);
            else
                app_fifo_put(&st_com_ringbuf_rx, p_event->data.rxtx.p_data[0]);
            nrf_drv_uart_rx(&st_com_instance, st_com_reqbuf_rx, 1);
            break;
        case NRF_DRV_UART_EVT_ERROR:
            nrf_drv_uart_errorsrc_get(&st_com_instance);
            nrf_drv_uart_rx(&st_com_instance, st_com_reqbuf_rx, 1);
            break;
        case NRF_DRV_UART_EVT_TX_DONE:
            break;
        default:
            break;
    }
}
#endif
int gl_com_init(unsigned char tx_pin, unsigned char rx_pin, unsigned long baudrate, int polling)
{
#ifdef INTERNAL_UART0_ENABLED
    if (st_com_initialized)
        return 1;
    if ((tx_pin >= GL_COM_PIN_DISCONNECTED) && (rx_pin >= GL_COM_PIN_DISCONNECTED))
        return 0;
    {
        int uses_handler;
        uint32_t ret;
        nrf_drv_uart_config_t config;
        config.pseltxd = tx_pin >= GL_COM_PIN_DISCONNECTED ? NRF_UART_PSEL_DISCONNECTED : tx_pin;
        config.pselrxd = rx_pin >= GL_COM_PIN_DISCONNECTED ? NRF_UART_PSEL_DISCONNECTED : rx_pin;
        config.pselcts = NRF_UART_PSEL_DISCONNECTED;
        config.pselrts = NRF_UART_PSEL_DISCONNECTED;
        config.p_context = 0;
        config.hwfc = NRF_UART_HWFC_DISABLED;
        config.parity = NRF_UART_PARITY_EXCLUDED;
        config.baudrate = st_com_get_baudrate(baudrate);
        config.interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY;
#if defined(NRF_DRV_UART_WITH_UARTE) && defined(NRF_DRV_UART_WITH_UART)
        config.use_easy_dma = false;
#endif
        if (!nrf_drv_clock_init_check())
            nrf_drv_clock_init();
        nrf_drv_clock_lfclk_request(0);
        while (!nrf_clock_lf_is_running())
            ;
#if APP_UART_ENABLED==1
        if (st_com_instance.inst_idx == APP_UART_DRIVER_INSTANCE)
            app_uart_close();
#endif
        app_fifo_init(&st_com_ringbuf_rx, st_com_buf_rx, sizeof(st_com_buf_rx));
        uses_handler = polling || (config.pselrxd == NRF_UART_PSEL_DISCONNECTED) ? 0 : 1;
        ret = nrf_drv_uart_init(&st_com_instance, &config, uses_handler ? st_com_event_handler : 0);
        if (ret == NRF_SUCCESS)
        {
            st_com_initialized = 1;
            st_com_tx_pin_present = config.pseltxd == NRF_UART_PSEL_DISCONNECTED ? 0 : 1;
            st_com_rx_pin_present = config.pselrxd == NRF_UART_PSEL_DISCONNECTED ? 0 : 1;
            st_com_uses_handler = uses_handler;
            if (st_com_rx_pin_present)
                nrf_drv_uart_rx_enable(&st_com_instance);
            if (st_com_uses_handler)
                ret = nrf_drv_uart_rx(&st_com_instance, st_com_reqbuf_rx, 1);
        }
        if (ret != NRF_SUCCESS)
        {
            st_com_initialized = 0;
            st_com_tx_pin_present = 0;
            st_com_rx_pin_present = 0;
            st_com_uses_handler = 0;
        }
        return st_com_initialized;
    }
#else
    return 0;
#endif
}
int gl_com_uninit(void)
{
#ifdef INTERNAL_UART0_ENABLED
    if (!st_com_initialized)
        return 1;
    {
        nrf_drv_uart_uninit(&st_com_instance);
        app_fifo_flush(&st_com_ringbuf_rx);
        st_com_initialized = 0;
        st_com_tx_pin_present = 0;
        st_com_rx_pin_present = 0;
        st_com_uses_handler = 0;
        return 1;
    }
#else
    return 1;
#endif
}
void gl_com_set_custom_receive_handler(void (*on_receive)(unsigned char))
{
#ifdef INTERNAL_UART0_ENABLED
    st_com_event_handler_custom_on_receive = on_receive;
#endif
}
unsigned long gl_com_read(void *buf, unsigned long len)
{
#ifdef INTERNAL_UART0_ENABLED
    uint32_t transferred = 0;
    if (!st_com_initialized)
        return 0;
    if (!st_com_rx_pin_present)
        return 0;
    if (!(buf && len))
        return 0;
    if (st_com_uses_handler)
    {
        transferred = (uint32_t)len;
        if (app_fifo_read(&st_com_ringbuf_rx, (uint8_t *)buf, &transferred) != NRF_SUCCESS)
            transferred = 0;
    }
    else
    {
        unsigned char *t = (unsigned char *)buf;
        nrf_drv_uart_errorsrc_get(&st_com_instance);
        while (nrf_drv_uart_rx_ready(&st_com_instance))
        {
            if (nrf_drv_uart_rx(&st_com_instance, t, 1) != NRF_SUCCESS)
                break;
            ++t;
            ++transferred;
            if (transferred == len)
                break;
        }
    }
    return (unsigned long)transferred;
#else
    return 0;
#endif
}
unsigned long gl_com_write(const void *buf, unsigned long len)
{
#ifdef INTERNAL_UART0_ENABLED
    uint32_t ret;
    unsigned long transferred = 0;
    if (!st_com_initialized)
        return 0;
    if (!st_com_tx_pin_present)
        return 0;
    if (!(buf && len))
        return 0;
    if (st_com_uses_handler)
    {
        while (nrf_drv_uart_tx_in_progress(&st_com_instance))
            __NOP();
        ret = fix_drv_uart_tx(&st_com_instance, buf, len);
        while (nrf_drv_uart_tx_in_progress(&st_com_instance))
            __NOP();
    }
    else
    {
        ret = fix_drv_uart_tx(&st_com_instance, buf, len);
    }
    if (ret == NRF_SUCCESS)
        return len;
#endif
    return 0;
}
void gl_com_flush(void)
{
#ifdef INTERNAL_UART0_ENABLED
    if (!st_com_initialized)
        return;
    if (!st_com_rx_pin_present)
        return;
    if (st_com_uses_handler)
    {
        app_fifo_flush(&st_com_ringbuf_rx);
    }
    else
    {
        unsigned char c;
        nrf_drv_uart_errorsrc_get(&st_com_instance);
        while (nrf_drv_uart_rx_ready(&st_com_instance))
            if (nrf_drv_uart_rx(&st_com_instance, &c, 1) != NRF_SUCCESS)
                break;
    }
#endif
}
/* UART 1 */
#ifdef INTERNAL_UART1_ENABLED
static const nrf_drv_uart_t st_com1_instance = NRF_DRV_UART_INSTANCE(1);
static int st_com1_initialized = 0;
static int st_com1_tx_pin_present = 0;
static int st_com1_rx_pin_present = 0;
static int st_com1_uses_handler = 0;
static unsigned char st_com1_buf_rx[512];
static app_fifo_t st_com1_ringbuf_rx = { .p_buf = st_com1_buf_rx, .buf_size_mask = sizeof(st_com1_buf_rx) - 1, .read_pos = 0, .write_pos = 0 };
static unsigned char st_com1_reqbuf_rx[1];
static void (*st_com1_event_handler_custom_on_receive)(unsigned char) = 0;
static void st_com1_event_handler(nrf_drv_uart_event_t *p_event, void *p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_UART_EVT_RX_DONE:
            if (st_com1_event_handler_custom_on_receive)
                st_com1_event_handler_custom_on_receive(p_event->data.rxtx.p_data[0]);
            else
            app_fifo_put(&st_com1_ringbuf_rx, p_event->data.rxtx.p_data[0]);
            nrf_drv_uart_rx(&st_com1_instance, st_com1_reqbuf_rx, 1);
            break;
        case NRF_DRV_UART_EVT_ERROR:
            nrf_drv_uart_errorsrc_get(&st_com1_instance);
            nrf_drv_uart_rx(&st_com1_instance, st_com1_reqbuf_rx, 1);
            break;
        case NRF_DRV_UART_EVT_TX_DONE:
            break;
        default:
            break;
    }
}
#endif
int gl_com1_init(unsigned char tx_pin, unsigned char rx_pin, unsigned long baudrate, int polling)
{
#ifdef INTERNAL_UART1_ENABLED
    if (st_com1_initialized)
        return 1;
    if ((tx_pin >= GL_COM_PIN_DISCONNECTED) && (rx_pin >= GL_COM_PIN_DISCONNECTED))
        return 0;
    {
        int uses_handler;
        uint32_t ret;
        nrf_drv_uart_config_t config;
        config.pseltxd = tx_pin >= GL_COM_PIN_DISCONNECTED ? NRF_UART_PSEL_DISCONNECTED : tx_pin;
        config.pselrxd = rx_pin >= GL_COM_PIN_DISCONNECTED ? NRF_UART_PSEL_DISCONNECTED : rx_pin;
        config.pselcts = NRF_UART_PSEL_DISCONNECTED;
        config.pselrts = NRF_UART_PSEL_DISCONNECTED;
        config.p_context = 0;
        config.hwfc = NRF_UART_HWFC_DISABLED;
        config.parity = NRF_UART_PARITY_EXCLUDED;
        config.baudrate = st_com_get_baudrate(baudrate);
        config.interrupt_priority = UART_DEFAULT_CONFIG_IRQ_PRIORITY;
#if defined(NRF_DRV_UART_WITH_UARTE) && defined(NRF_DRV_UART_WITH_UART)
        config.use_easy_dma = true;
#endif
        if (!nrf_drv_clock_init_check())
            nrf_drv_clock_init();
        nrf_drv_clock_lfclk_request(0);
        while (!nrf_clock_lf_is_running())
            ;
#if APP_UART_ENABLED==1
        if (st_com1_instance.inst_idx == APP_UART_DRIVER_INSTANCE)
            app_uart_close();
#endif
        app_fifo_init(&st_com1_ringbuf_rx, st_com1_buf_rx, sizeof(st_com1_buf_rx));
        uses_handler = polling || (config.pselrxd == NRF_UART_PSEL_DISCONNECTED) ? 0 : 1;
        ret = nrf_drv_uart_init(&st_com1_instance, &config, uses_handler ? st_com1_event_handler : 0);
        if (ret == NRF_SUCCESS)
        {
            st_com1_initialized = 1;
            st_com1_tx_pin_present = config.pseltxd == NRF_UART_PSEL_DISCONNECTED ? 0 : 1;
            st_com1_rx_pin_present = config.pselrxd == NRF_UART_PSEL_DISCONNECTED ? 0 : 1;
            st_com1_uses_handler = uses_handler;
            if (st_com1_rx_pin_present)
                nrf_drv_uart_rx_enable(&st_com1_instance);
            if (st_com1_uses_handler)
                ret = nrf_drv_uart_rx(&st_com1_instance, st_com1_reqbuf_rx, sizeof(st_com1_reqbuf_rx));
        }
        if (ret != NRF_SUCCESS)
        {
            st_com1_initialized = 0;
            st_com1_tx_pin_present = 0;
            st_com1_rx_pin_present = 0;
            st_com1_uses_handler = 0;
        }
        return st_com1_initialized;
    }
#else
    return 0;
#endif
}
int gl_com1_uninit(void)
{
#ifdef INTERNAL_UART1_ENABLED
    if (!st_com1_initialized)
        return 1;
    {
        nrf_drv_uart_uninit(&st_com1_instance);
        app_fifo_flush(&st_com1_ringbuf_rx);
        st_com1_initialized = 0;
        st_com1_tx_pin_present = 0;
        st_com1_rx_pin_present = 0;
        st_com1_uses_handler = 0;
        return 1;
    }
#else
    return 1;
#endif
}
void gl_com1_set_custom_receive_handler(void (*on_receive)(unsigned char))
{
#ifdef INTERNAL_UART1_ENABLED
    st_com1_event_handler_custom_on_receive = on_receive;
#endif
}
unsigned long gl_com1_read(void *buf, unsigned long len)
{
#ifdef INTERNAL_UART1_ENABLED
    uint32_t transferred = 0;
    if (!st_com1_initialized)
        return 0;
    if (!st_com1_rx_pin_present)
        return 0;
    if (!(buf && len))
        return 0;
    if (st_com1_uses_handler)
    {
        transferred = (uint32_t)len;
        if (app_fifo_read(&st_com1_ringbuf_rx, (uint8_t *)buf, &transferred) != NRF_SUCCESS)
            transferred = 0;
    }
    else
    {
        unsigned char *t = (unsigned char *)buf;
        nrf_drv_uart_errorsrc_get(&st_com1_instance);
        while (nrf_drv_uart_rx_ready(&st_com1_instance))
        {
            if (nrf_drv_uart_rx(&st_com1_instance, t, 1) != NRF_SUCCESS)
                break;
            ++t;
            ++transferred;
            if (transferred == len)
                break;
        }
    }
    return (unsigned long)transferred;
#else
    return 0;
#endif
}
unsigned long gl_com1_write(const void *buf, unsigned long len)
{
#ifdef INTERNAL_UART1_ENABLED
    uint32_t ret;
    unsigned long transferred = 0;
    if (!st_com1_initialized)
        return 0;
    if (!st_com1_tx_pin_present)
        return 0;
    if (!(buf && len))
        return 0;
    if (st_com1_uses_handler)
    {
        while (nrf_drv_uart_tx_in_progress(&st_com1_instance))
            __NOP();
        ret = fix_drv_uart_tx(&st_com1_instance, buf, len);
        while (nrf_drv_uart_tx_in_progress(&st_com1_instance))
            __NOP();
    }
    else
    {
        ret = fix_drv_uart_tx(&st_com1_instance, buf, len);
    }
    if (ret == NRF_SUCCESS)
        return len;
#endif
    return 0;
}
void gl_com1_flush(void)
{
#ifdef INTERNAL_UART1_ENABLED
    if (!st_com1_initialized)
        return;
    if (!st_com1_rx_pin_present)
        return;
    if (st_com1_uses_handler)
    {
        app_fifo_flush(&st_com1_ringbuf_rx);
    }
    else
    {
        unsigned char c;
        nrf_drv_uart_errorsrc_get(&st_com1_instance);
        while (nrf_drv_uart_rx_ready(&st_com1_instance))
            if (nrf_drv_uart_rx(&st_com1_instance, &c, 1) != NRF_SUCCESS)
                break;
    }
#endif
}
/**/
