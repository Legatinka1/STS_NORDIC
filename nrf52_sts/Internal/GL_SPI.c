#include "GL_SPI.h"
#include <stdint.h>
#include <stdbool.h>
#include <nrfx.h>

/* SPI */
#if (SPI1_ENABLED==1) || (SPI2_ENABLED==1)
#include "nrf_gpio.h"
static void st_spim_prepare_ss_pin(uint8_t ss_pin)
{
    if (ss_pin < NUMBER_OF_PINS)
    {
        nrf_gpio_pin_dir_set(ss_pin, NRF_GPIO_PIN_DIR_OUTPUT);
        nrf_gpio_pin_set(ss_pin);
    }
}
static void st_spim_gpio_pin_set(uint8_t ss_pin)
{
    if (ss_pin < NUMBER_OF_PINS)
        nrf_gpio_pin_set(ss_pin);
}
static void st_spim_gpio_pin_clear(uint8_t ss_pin)
{
    if (ss_pin < NUMBER_OF_PINS)
        nrf_gpio_pin_clear(ss_pin);
}
#include "nrf_drv_spi.h"
__STATIC_INLINE
ret_code_t fix_drv_spi_transfer(nrf_drv_spi_t const * const p_instance,
                                uint8_t const * p_tx_buffer,
                                uint32_t        tx_buffer_length,
                                uint8_t       * p_rx_buffer,
                                uint32_t        rx_buffer_length)
/*
{
    ret_code_t result = 0;
    if (NRF_DRV_SPI_USE_SPIM)
    {
    #ifdef SPIM_PRESENT
        nrfx_spim_xfer_desc_t const spim_xfer_desc =
        {
            .p_tx_buffer = p_tx_buffer,
            .tx_length   = tx_buffer_length,
            .p_rx_buffer = p_rx_buffer,
            .rx_length   = rx_buffer_length,
        };
        result = nrfx_spim_xfer(&p_instance->u.spim, &spim_xfer_desc, 0);
    #endif
    }
    else if (NRF_DRV_SPI_USE_SPI)
    {
    #ifdef SPI_PRESENT
        nrfx_spi_xfer_desc_t const spi_xfer_desc =
        {
            .p_tx_buffer = p_tx_buffer,
            .tx_length   = tx_buffer_length,
            .p_rx_buffer = p_rx_buffer,
            .rx_length   = rx_buffer_length,
        };
        result = nrfx_spi_xfer(&p_instance->u.spi, &spi_xfer_desc, 0);
    #endif
    }
    return result;
}
*/
{
    if ((tx_buffer_length <= 255) && (rx_buffer_length <= 255))
        return nrf_drv_spi_transfer(p_instance, p_tx_buffer, tx_buffer_length, p_rx_buffer, rx_buffer_length);
    {
        uint32_t ret;
        uint32_t tx_buffer_length_rest = tx_buffer_length, rx_buffer_length_rest = rx_buffer_length, tx_buffer_length_chunk, rx_buffer_length_chunk;
        uint8_t *p_tx_buffer_pos = (uint8_t *)p_tx_buffer, *p_rx_buffer_pos = p_rx_buffer;
        while (tx_buffer_length_rest || rx_buffer_length_rest)
        {
            tx_buffer_length_chunk = tx_buffer_length_rest > 255 ? 255 : tx_buffer_length_rest;
            rx_buffer_length_chunk = rx_buffer_length_rest > 255 ? 255 : rx_buffer_length_rest;
            ret = nrf_drv_spi_transfer(p_instance, p_tx_buffer_pos, tx_buffer_length_chunk, p_rx_buffer_pos, rx_buffer_length_chunk);
            if (ret != NRF_SUCCESS)
                return ret;
            tx_buffer_length_rest -= tx_buffer_length_chunk;
            rx_buffer_length_rest -= rx_buffer_length_chunk;
            p_tx_buffer_pos += tx_buffer_length_chunk;
            p_rx_buffer_pos += rx_buffer_length_chunk;
        }
        return NRF_SUCCESS;
    }
}
static int st_spim_init(nrf_drv_spi_t const * const p_instance, uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t mode)
{
    nrf_drv_spi_config_t spi_config;
    spi_config.sck_pin      = sck_pin;
    spi_config.mosi_pin     = mosi_pin;
    spi_config.miso_pin     = miso_pin;
    spi_config.ss_pin       = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.irq_priority = APP_IRQ_PRIORITY_LOWEST;
    spi_config.orc          = 0xFF;
    spi_config.frequency    = NRF_DRV_SPI_FREQ_8M;
    switch (mode)
    {
        case 0:
            spi_config.mode = NRF_DRV_SPI_MODE_0;
            break;
        case 1:
            spi_config.mode = NRF_DRV_SPI_MODE_1;
            break;
        case 2:
            spi_config.mode = NRF_DRV_SPI_MODE_2;
            break;
        case 3:
            spi_config.mode = NRF_DRV_SPI_MODE_3;
            break;
        default:
            spi_config.mode = NRF_DRV_SPI_MODE_0;
            break;
    }
    spi_config.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
    if (nrf_drv_spi_init(p_instance, &spi_config, NULL, NULL) != NRF_SUCCESS)
        return 0;
    return 1;
}
#endif
/* SPI 1 */
#if SPI1_ENABLED==1
#define INTERNAL_SPI1_ENABLED
#define ST_SPI_INSTANCE 1
static const nrf_drv_spi_t st_spi = NRF_DRV_SPI_INSTANCE(ST_SPI_INSTANCE);
#endif
static int st_spi_initialized = 0;
void gl_spi_init(uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t mode)
{
    if (st_spi_initialized)
        return;
#ifdef INTERNAL_SPI1_ENABLED
    st_spi_initialized = st_spim_init(&st_spi, mosi_pin, miso_pin, sck_pin, mode);
#endif
}
#include "nrf_gpio.h"
void gl_spi_prepare_ss_pin(uint8_t ss_pin)
{
#ifdef INTERNAL_SPI1_ENABLED
    st_spim_prepare_ss_pin(ss_pin);
#endif
}
uint8_t gl_spi_read_reg_byte(uint8_t ss_pin, uint8_t reg)
{
#ifdef INTERNAL_SPI1_ENABLED
    uint8_t rreg = reg | 0x80, data[2] = { 0, 0 };
    st_spim_gpio_pin_clear(ss_pin);
    fix_drv_spi_transfer(&st_spi, &rreg, 1, data, 2);
    st_spim_gpio_pin_set(ss_pin);
    return data[1];
#else
    return 0;
#endif
}
void gl_spi_write_reg_byte(uint8_t ss_pin, uint8_t reg, uint8_t value)
{
#ifdef INTERNAL_SPI1_ENABLED
    uint8_t data[2] = { reg & 0x7F, value };
    st_spim_gpio_pin_clear(ss_pin);
    fix_drv_spi_transfer(&st_spi, data, 2, 0, 0);
    st_spim_gpio_pin_set(ss_pin);
#endif
}
#define SPI_STAT_BUF_SIZE 32
#ifdef INTERNAL_SPI1_ENABLED
static uint8_t st_spi_buf[SPI_STAT_BUF_SIZE + 1];
#endif
uint32_t gl_spi_read_reg(uint8_t ss_pin, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI1_ENABLED
    uint8_t rreg = reg | 0x80;
    int i;
    if (!buf)
        return NRF_ERROR_NULL;
    st_spim_gpio_pin_clear(ss_pin);
    if (len <= SPI_STAT_BUF_SIZE)
    {
        ret = fix_drv_spi_transfer(&st_spi, &rreg, 1, st_spi_buf, len + 1);
        for (i = 0; i < len; i++)
            buf[i] = st_spi_buf[i + 1];
    }
    else
    {
        ret = fix_drv_spi_transfer(&st_spi, &rreg, 1, 0, 0);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi, 0, 0, buf, len);
    }
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi_write_reg(uint8_t ss_pin, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI1_ENABLED
    uint8_t wreg = reg & 0x7F;
    int i;
    if (!buf)
        return NRF_ERROR_NULL;
    st_spim_gpio_pin_clear(ss_pin);
    if (len <= SPI_STAT_BUF_SIZE)
    {
        st_spi_buf[0] = wreg;
        for (i = 0; i < len; i++)
            st_spi_buf[i + 1] = buf[i];
        ret = fix_drv_spi_transfer(&st_spi, st_spi_buf, len + 1, 0, 0);
    }
    else
    {
        ret = fix_drv_spi_transfer(&st_spi, &wreg, 1, 0, 0);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi, buf, len, 0, 0);
    }
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi_write_and_or_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint32_t len_to_write, uint8_t *buf_to_read, uint32_t len_to_read)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI1_ENABLED
    if (((!buf_to_write) && (!buf_to_read)) || ((!len_to_write) && (!len_to_read)) ||
        ((!buf_to_write) && len_to_write) || ((!buf_to_read) && len_to_read))
        return NRF_ERROR_INVALID_PARAM;
    st_spim_gpio_pin_clear(ss_pin);
    if (len_to_write)
        ret = fix_drv_spi_transfer(&st_spi, buf_to_write, len_to_write, 0, 0);
    else
        ret = NRF_SUCCESS;
    if (len_to_read)
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi, 0, 0, buf_to_read, len_to_read);
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi_write_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint8_t *buf_to_read, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI1_ENABLED
    if (((!buf_to_write) && (!buf_to_read)) || (!len))
        return NRF_ERROR_INVALID_PARAM;
    st_spim_gpio_pin_clear(ss_pin);
    ret = fix_drv_spi_transfer(&st_spi, buf_to_write, buf_to_write ? len : 0, buf_to_read, buf_to_read ? len : 0);
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
/* SPI2 */
#if SPI2_ENABLED==1
#define INTERNAL_SPI2_ENABLED
#define ST_SPI2_INSTANCE 2
static const nrf_drv_spi_t st_spi2 = NRF_DRV_SPI_INSTANCE(ST_SPI2_INSTANCE);
#endif
static int st_spi2_initialized = 0;
void gl_spi2_init(uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t mode)
{
    if (st_spi2_initialized)
        return;
#ifdef INTERNAL_SPI2_ENABLED
    st_spi2_initialized = st_spim_init(&st_spi2, mosi_pin, miso_pin, sck_pin, mode);
#endif
}
#include "nrf_gpio.h"
void gl_spi2_prepare_ss_pin(uint8_t ss_pin)
{
#ifdef INTERNAL_SPI2_ENABLED
    st_spim_prepare_ss_pin(ss_pin);
#endif
}
uint8_t gl_spi2_read_reg_byte(uint8_t ss_pin, uint8_t reg)
{
#ifdef INTERNAL_SPI2_ENABLED
    uint8_t rreg = reg | 0x80, data[2] = { 0, 0 };
    st_spim_gpio_pin_clear(ss_pin);
    fix_drv_spi_transfer(&st_spi2, &rreg, 1, data, 2);
    st_spim_gpio_pin_set(ss_pin);
    return data[1];
#else
    return 0;
#endif
}
void gl_spi2_write_reg_byte(uint8_t ss_pin, uint8_t reg, uint8_t value)
{
#ifdef INTERNAL_SPI2_ENABLED
    uint8_t data[2] = { reg & 0x7F, value };
    st_spim_gpio_pin_clear(ss_pin);
    fix_drv_spi_transfer(&st_spi2, data, 2, 0, 0);
    st_spim_gpio_pin_set(ss_pin);
#endif
}
#define SPI2_STAT_BUF_SIZE 32
#ifdef INTERNAL_SPI2_ENABLED
static uint8_t st_spi2_buf[SPI2_STAT_BUF_SIZE + 1];
#endif
uint32_t gl_spi2_read_reg(uint8_t ss_pin, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI2_ENABLED
    uint8_t rreg = reg | 0x80;
    int i;
    if (!buf)
        return NRF_ERROR_NULL;
    st_spim_gpio_pin_clear(ss_pin);
    if (len <= SPI2_STAT_BUF_SIZE)
    {
        ret = fix_drv_spi_transfer(&st_spi2, &rreg, 1, st_spi2_buf, len + 1);
        for (i = 0; i < len; i++)
            buf[i] = st_spi2_buf[i + 1];
    }
    else
    {
        ret = fix_drv_spi_transfer(&st_spi2, &rreg, 1, 0, 0);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi2, 0, 0, buf, len);
    }
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi2_write_reg(uint8_t ss_pin, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI2_ENABLED
    uint8_t wreg = reg & 0x7F;
    int i;
    if (!buf)
        return NRF_ERROR_NULL;
    st_spim_gpio_pin_clear(ss_pin);
    if (len <= SPI2_STAT_BUF_SIZE)
    {
        st_spi2_buf[0] = wreg;
        for (i = 0; i < len; i++)
            st_spi2_buf[i + 1] = buf[i];
        ret = fix_drv_spi_transfer(&st_spi2, st_spi2_buf, len + 1, 0, 0);
    }
    else
    {
        ret = fix_drv_spi_transfer(&st_spi2, &wreg, 1, 0, 0);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi2, buf, len, 0, 0);
    }
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi2_write_and_or_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint32_t len_to_write, uint8_t *buf_to_read, uint32_t len_to_read)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI2_ENABLED
    if (((!buf_to_write) && (!buf_to_read)) || ((!len_to_write) && (!len_to_read)) ||
        ((!buf_to_write) && len_to_write) || ((!buf_to_read) && len_to_read))
        return NRF_ERROR_INVALID_PARAM;
    st_spim_gpio_pin_clear(ss_pin);
    if (len_to_write)
        ret = fix_drv_spi_transfer(&st_spi2, buf_to_write, len_to_write, 0, 0);
    else
        ret = NRF_SUCCESS;
    if (len_to_read)
        if (ret == NRF_SUCCESS)
            ret = fix_drv_spi_transfer(&st_spi2, 0, 0, buf_to_read, len_to_read);
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
uint32_t gl_spi2_write_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint8_t *buf_to_read, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_SPI2_ENABLED
    if (((!buf_to_write) && (!buf_to_read)) || (!len))
        return NRF_ERROR_INVALID_PARAM;
    st_spim_gpio_pin_clear(ss_pin);
    ret = fix_drv_spi_transfer(&st_spi2, buf_to_write, buf_to_write ? len : 0, buf_to_read, buf_to_read ? len : 0);
    st_spim_gpio_pin_set(ss_pin);
#endif
    return ret;
}
/* QSPI */
#if QSPI_ENABLED==1
#define INTERNAL_QSPI_ENABLED
#include "nrf_drv_qspi.h"
#endif
static volatile bool st_qspi_finished = false;
#define ST_QSPI_WAIT_FOR_PERIPH() do { \
        while (!st_qspi_finished) {} \
        st_qspi_finished = false;    \
    } while (0)
#ifdef INTERNAL_QSPI_ENABLED
static void st_qspi_handler(nrf_drv_qspi_evt_t event, void *p_context)
{
    UNUSED_PARAMETER(event);
    UNUSED_PARAMETER(p_context);
    st_qspi_finished = true;
}
#endif
#define QSPI_STD_CMD_WRSR   0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST    0x99
static void st_qspi_configure_memory()
{
#ifdef INTERNAL_QSPI_ENABLED
    uint8_t temporary = 0x40;
    uint32_t err_code;
    nrf_qspi_cinstr_conf_t cinstr_cfg = {
        .opcode    = QSPI_STD_CMD_RSTEN,
        .length    = NRF_QSPI_CINSTR_LEN_1B,
        .io2_level = true,
        .io3_level = true,
        .wipwait   = true,
        .wren      = true
    };
    // Send reset enable
    err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    // Send reset command
    cinstr_cfg.opcode = QSPI_STD_CMD_RST;
    err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    // Switch to qspi mode
    cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
    cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
    err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}
static bool st_qspi_initialized = false;
void gl_qspi_init(uint8_t sck_pin, uint8_t csn_pin, uint8_t io0_pin, uint8_t io1_pin, uint8_t io2_pin, uint8_t io3_pin)
{
    if (st_qspi_initialized)
    {
        return;
    }
    else
    {
#ifdef INTERNAL_QSPI_ENABLED
        nrf_drv_qspi_config_t qspi_config/* = NRF_DRV_QSPI_DEFAULT_CONFIG*/;
        qspi_config.xip_offset = NRFX_QSPI_CONFIG_XIP_OFFSET;
        qspi_config.pins.sck_pin = sck_pin;
        qspi_config.pins.csn_pin = csn_pin;
        qspi_config.pins.io0_pin = io0_pin;
        qspi_config.pins.io1_pin = io1_pin;
        qspi_config.pins.io2_pin = io2_pin;
        qspi_config.pins.io3_pin = io3_pin;
        qspi_config.prot_if.readoc    = (nrf_qspi_readoc_t)NRFX_QSPI_CONFIG_READOC;
        qspi_config.prot_if.writeoc   = (nrf_qspi_writeoc_t)NRFX_QSPI_CONFIG_WRITEOC;
        qspi_config.prot_if.addrmode  = (nrf_qspi_addrmode_t)NRFX_QSPI_CONFIG_ADDRMODE;
        qspi_config.prot_if.dpmconfig = false;
        qspi_config.phy_if.sck_delay  = (uint8_t)NRFX_QSPI_CONFIG_SCK_DELAY;
        qspi_config.phy_if.dpmen      = false;
        qspi_config.phy_if.spi_mode   = (nrf_qspi_spi_mode_t)NRFX_QSPI_CONFIG_MODE;
        qspi_config.phy_if.sck_freq   = (nrf_qspi_frequency_t)NRFX_QSPI_CONFIG_FREQUENCY;
        qspi_config.irq_priority = (uint8_t)NRFX_QSPI_CONFIG_IRQ_PRIORITY;
        if (nrf_drv_qspi_init(&qspi_config, st_qspi_handler, NULL) != NRF_SUCCESS)
            return;
        st_qspi_configure_memory();
        st_qspi_initialized = true;
#endif
    }
}
uint32_t gl_qspi_erase(void)
{
    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_QSPI_ENABLED
    st_qspi_finished = false;
    err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, 0);
    APP_ERROR_CHECK(err_code);
    ST_QSPI_WAIT_FOR_PERIPH();
#endif
    return err_code;
}
uint32_t gl_qspi_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_QSPI_ENABLED
    if (!buf)
        return NRF_ERROR_NULL;
    st_qspi_finished = false;
    err_code = nrf_drv_qspi_write(buf, len, addr);
    APP_ERROR_CHECK(err_code);
    ST_QSPI_WAIT_FOR_PERIPH();
#endif
    return err_code;
}
uint32_t gl_qspi_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t err_code = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_QSPI_ENABLED
    if (!buf)
        return NRF_ERROR_NULL;
    st_qspi_finished = false;
    err_code = nrf_drv_qspi_read(buf, len, addr);
    APP_ERROR_CHECK(err_code);
    ST_QSPI_WAIT_FOR_PERIPH();
#endif
    return err_code;
}
/**/
