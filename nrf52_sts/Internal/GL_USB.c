#include "GL_USB.h"

#include <nrfx.h>
#if USBD_ENABLED==1
#define INTERNAL_USBD_ENABLED
#endif

#ifdef INTERNAL_USBD_ENABLED

// Attention: have to set "NRFX_POWER_ENABLED = 1" and "POWER_ENABLED = 1" and call nrf_drv_power_init(0) before SoftDevice initialization!
//#if NRF_MODULE_ENABLED(POWER)
//    nrf_drv_power_init(0); // Need for USB
//#endif

/* Storage and Status */

#include "app_fifo.h"

static uint32_t st_fifo_length(app_fifo_t * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

static uint32_t st_fifo_free(app_fifo_t * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    uint32_t len = p_fifo->write_pos - tmp;
    return p_fifo->buf_size_mask - len + 1;
}

static uint32_t st_fifo_free_continuous(app_fifo_t * p_fifo)
{
    uint32_t r_pos = p_fifo->read_pos;
    uint32_t w_pos = p_fifo->write_pos;
    uint32_t len = w_pos - r_pos;
    if (len > p_fifo->buf_size_mask)
        return 0;
    r_pos &= p_fifo->buf_size_mask;
    w_pos &= p_fifo->buf_size_mask;
    if (r_pos > w_pos)
        return r_pos - w_pos;
    return p_fifo->buf_size_mask - w_pos + 1;
}

static uint8_t *st_fifo_write_pointer(app_fifo_t * p_fifo)
{
    return p_fifo->p_buf + (p_fifo->write_pos & p_fifo->buf_size_mask);
}

static unsigned char st_buf_rx[512];
static app_fifo_t st_ringbuf_rx = { .p_buf = st_buf_rx, .buf_size_mask = sizeof(st_buf_rx) - 1, .read_pos = 0, .write_pos = 0 };
static int st_usbd_ready = 0;
static int st_usbd_tx_done = 0;
static unsigned char st_c_rx;
#define ST_C_RX_DUMMY_SIZE 64
static unsigned char st_c_rx_dummy[ST_C_RX_DUMMY_SIZE];

/* Core */

#include "nrf_drv_clock.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

//#define READ_SIZE 1
//
//static char m_rx_buffer[READ_SIZE];
//static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
//static bool m_send_flag = 0;

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    //app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            //bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            //ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
            //                                       m_rx_buffer,
            //                                       READ_SIZE);
            //UNUSED_VARIABLE(ret);
            //
            app_usbd_cdc_acm_read(&m_app_cdc_acm, &st_c_rx, 1);
            //
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            //bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            //bsp_board_led_invert(LED_CDC_ACM_TX);
            //
            st_usbd_tx_done = 1;
            //
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            //ret_code_t ret;
            //NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
            //do
            //{
            //    /*Get amount of data transfered*/
            //    size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
            //    //NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);
            //
            //    /* Fetch data until internal buffer is empty */
            //    ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
            //                                m_rx_buffer,
            //                                READ_SIZE);
            //} while (ret == NRF_SUCCESS);
            //
            {
                uint32_t avail = st_fifo_free(&st_ringbuf_rx);
                uint32_t stored = app_usbd_cdc_acm_bytes_stored(&m_app_cdc_acm);
                uint32_t len;
                if (avail)
                {
                    app_fifo_put(&st_ringbuf_rx, st_c_rx);
                    --avail;
                    if (avail && stored)
                    {
                        uint32_t count = avail >= stored ? stored : avail;
                        uint32_t subcount = st_fifo_free_continuous(&st_ringbuf_rx);
                        if (subcount > count)
                            subcount = count;
                        stored -= count;
                        avail -= count;
                        app_usbd_cdc_acm_read(&m_app_cdc_acm, st_fifo_write_pointer(&st_ringbuf_rx), subcount);
                        st_ringbuf_rx.write_pos += subcount;
                        count -= subcount;
                        if (count)
                        {
                            app_usbd_cdc_acm_read(&m_app_cdc_acm, st_ringbuf_rx.p_buf, count);
                            st_ringbuf_rx.write_pos += count;
                        }
                    }
                }
                while (stored)
                {
                    len = stored >= ST_C_RX_DUMMY_SIZE ? ST_C_RX_DUMMY_SIZE : stored;
                    stored -= len;
                    app_usbd_cdc_acm_read(&m_app_cdc_acm, st_c_rx_dummy, len);
                }
                while (app_usbd_cdc_acm_read(&m_app_cdc_acm, &st_c_rx, 1) == NRF_SUCCESS)
                {
                    if (avail)
                    {
                        --avail;
                        app_fifo_put(&st_ringbuf_rx, st_c_rx);
                    }
                }
            }
            //

            //bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            //bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            //bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            //bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            //NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            //NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            //
            st_usbd_ready = 0;
            app_fifo_flush(&st_ringbuf_rx);
            //
            break;
        case APP_USBD_EVT_POWER_READY:
            //NRF_LOG_INFO("USB ready");
            app_usbd_start();
            //
            st_usbd_ready = 1;
            app_fifo_flush(&st_ringbuf_rx);
            //
            break;
        default:
            break;
    }
}

#endif

/* API */

static int st_usbd_initialized = 0;

void gl_usb_init(void)
{
    if (st_usbd_initialized)
        return;
    {
#ifdef INTERNAL_USBD_ENABLED
        ret_code_t ret;
        static const app_usbd_config_t usbd_config = {
            .ev_state_proc = usbd_user_ev_handler
        };
        //
        st_usbd_initialized = 1;
        app_fifo_init(&st_ringbuf_rx, st_buf_rx, sizeof(st_buf_rx));
        //

        app_usbd_serial_num_generate();

        ret = nrf_drv_clock_init();
        if (ret != NRF_ERROR_MODULE_ALREADY_INITIALIZED)
            APP_ERROR_CHECK(ret);

        ret = app_usbd_init(&usbd_config);
        APP_ERROR_CHECK(ret);

        app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
        ret = app_usbd_class_append(class_cdc_acm);
        APP_ERROR_CHECK(ret);
#endif
    }
}

static int st_power_events_disabled = 1;

void gl_usb_process(void)
{
#ifdef INTERNAL_USBD_ENABLED
    if (!st_usbd_initialized)
        return;
    if (st_power_events_disabled)
    {
        st_power_events_disabled = 0;
        app_usbd_power_events_enable();
    }
    while (app_usbd_event_queue_process())
        ;
#endif
}

int gl_usb_is_connected(void)
{
#ifdef INTERNAL_USBD_ENABLED
    return st_usbd_ready;
#else
    return 0;
#endif
}

unsigned short gl_usb_write(const void *data, unsigned short len, int optionally)
{
#ifdef INTERNAL_USBD_ENABLED
    ret_code_t ret;
    unsigned short len_ret;
    if (!(st_usbd_ready && data && len))
        return 0;
    st_usbd_tx_done = 0;
    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, data, len);
    if (ret == NRF_SUCCESS)
    {
        while (st_usbd_ready && (!st_usbd_tx_done))
            app_usbd_event_queue_process();
        return len;
    }
    if (optionally)
        return 0;
    while ((ret == NRF_ERROR_BUSY) && st_usbd_ready)
    {
        st_usbd_tx_done = 0;
        ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, data, len);
        if (ret == NRF_SUCCESS)
        {
            while (st_usbd_ready && (!st_usbd_tx_done))
                app_usbd_event_queue_process();
            return len;
        }
    }
#endif
    return 0;
}

unsigned short gl_usb_get_readable_count(void)
{
#ifdef INTERNAL_USBD_ENABLED
    return (unsigned short)st_fifo_length(&st_ringbuf_rx);
#else
    return 0;
#endif
}

unsigned short gl_usb_read(void *data, unsigned short count, int peek)
{
#ifdef INTERNAL_USBD_ENABLED
    unsigned short ret;
    uint32_t old_read_pos = st_ringbuf_rx.read_pos, real_count = count;
    ret = app_fifo_read(&st_ringbuf_rx, (uint8_t *)data, &real_count) == NRF_SUCCESS ? (unsigned short)real_count : 0;
    if (peek)
        st_ringbuf_rx.read_pos = old_read_pos;
    return ret;
#else
    return 0;
#endif
}

unsigned short gl_usb_peek(unsigned short pos, unsigned char *data)
{
#ifdef INTERNAL_USBD_ENABLED
    if (!data)
        return 0;
    return app_fifo_peek(&st_ringbuf_rx, pos, data) == NRF_SUCCESS ? 1 : 0;
#else
    if (data)
        *data = 0;
    return 0;
#endif
}

unsigned short gl_usb_get_free_space_for_read(void)
{
#ifdef INTERNAL_USBD_ENABLED
    return (unsigned short)st_fifo_free(&st_ringbuf_rx);
#else
    return 0;
#endif
}

void gl_usb_flush(void)
{
#ifdef INTERNAL_USBD_ENABLED
    gl_usb_process();
    app_fifo_flush(&st_ringbuf_rx);
#endif
}
