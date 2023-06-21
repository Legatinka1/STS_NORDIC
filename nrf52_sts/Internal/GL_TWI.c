#include "GL_TWI.h"
#include <stdint.h>
#include <stdbool.h>
#include <nrfx.h>

/* TWI */
#if (TWI0_ENABLED==1) || (TWI1_ENABLED==1)
#include "nrf_drv_twi.h"
__STATIC_INLINE
ret_code_t fix_drv_twi_tx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t const *       p_data,
                          uint32_t              length,
                          bool                  no_stop)
{
    ret_code_t result = 0;
    if (NRF_DRV_TWI_USE_TWIM)
    {
        result = nrfx_twim_tx(&p_instance->u.twim,
                                address, p_data, length, no_stop);
    }
    else if (NRF_DRV_TWI_USE_TWI)
    {
        result = nrfx_twi_tx(&p_instance->u.twi,
                               address, p_data, length, no_stop);
    }
    return result;
}

__STATIC_INLINE
ret_code_t fix_drv_twi_rx(nrf_drv_twi_t const * p_instance,
                          uint8_t               address,
                          uint8_t *             p_data,
                          uint32_t              length)
{
    ret_code_t result = 0;
    if (NRF_DRV_TWI_USE_TWIM)
    {
        result = nrfx_twim_rx(&p_instance->u.twim,
                                address, p_data, length);
    }
    else if (NRF_DRV_TWI_USE_TWI)
    {
        result = nrfx_twi_rx(&p_instance->u.twi,
                               address, p_data, length);
    }
    return result;
}
static bool st_twim_init(nrf_drv_twi_t const * const p_instance, uint8_t sda_pin, uint8_t scl_pin)
{
    nrf_drv_twi_config_t config;
    config.scl                = scl_pin;
    config.sda                = sda_pin;
    config.frequency          = NRF_DRV_TWI_FREQ_400K;
    config.interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
    config.clear_bus_init     = false;
    config.hold_bus_uninit    = false;
    if (nrf_drv_twi_init(p_instance, &config, NULL, NULL) != NRF_SUCCESS)
        return false;
    nrf_drv_twi_enable(p_instance);
    return true;
}
#endif
/* TWI 0 */
#if TWI0_ENABLED==1
#define INTERNAL_TWI0_ENABLED
#define ST_TWI_INSTANCE 0
static const nrf_drv_twi_t st_twi = NRF_DRV_TWI_INSTANCE(ST_TWI_INSTANCE);
#endif
static bool st_twi_initialized = false;
void gl_twi_init(uint8_t sda_pin, uint8_t scl_pin)
{
#ifdef INTERNAL_TWI0_ENABLED
    if (st_twi_initialized)
        return;
    else
        st_twi_initialized = st_twim_init(&st_twi, sda_pin, scl_pin);
#endif
}
uint8_t gl_twi_read_reg_byte(uint8_t address, uint8_t reg)
{
    uint8_t ret = 0;
#ifdef INTERNAL_TWI0_ENABLED
    if (fix_drv_twi_tx(&st_twi, address, &reg, 1, true) == NRF_SUCCESS)
        fix_drv_twi_rx(&st_twi, address, &ret, 1);
#endif
    return ret;
}
void gl_twi_write_reg_byte(uint8_t address, uint8_t reg, uint8_t value)
{
#ifdef INTERNAL_TWI0_ENABLED
    uint8_t data[2] = { reg, value };
    fix_drv_twi_tx(&st_twi, address, data, 2, false);
#endif
}
uint32_t gl_twi_read_reg(uint8_t address, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_TWI0_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    ret = fix_drv_twi_tx(&st_twi, address, &reg, 1, true);
    if (ret == NRF_SUCCESS)
        ret = fix_drv_twi_rx(&st_twi, address, buf, (uint8_t)len);
#endif
    return ret;
}
#define TWI_STAT_BUF_SIZE 32
#ifdef INTERNAL_TWI0_ENABLED
static uint8_t st_twi_buf[TWI_STAT_BUF_SIZE + 1];
#endif
uint32_t gl_twi_write_reg(uint8_t address, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_TWI0_ENABLED
    int i;
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    if (len <= TWI_STAT_BUF_SIZE)
    {
        st_twi_buf[0] = reg;
        for (i = 0; i < len; i++)
            st_twi_buf[i + 1] = buf[i];
        ret = fix_drv_twi_tx(&st_twi, address, st_twi_buf, (uint8_t)(len + 1), false);
    }
    else
    {
        ret = fix_drv_twi_tx(&st_twi, address, &reg, 1, true);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_twi_tx(&st_twi, address, buf, (uint8_t)len, false);
    }
#endif
    return ret;
}
int gl_twi_is_present(uint8_t address)
{
#ifdef INTERNAL_TWI0_ENABLED
    return fix_drv_twi_tx(&st_twi, address, 0, 0, false) ? 0 : 1;
#else
    return 0;
#endif
}
uint32_t gl_twi_read(uint8_t address, uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI0_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_rx(&st_twi, address, buf, (uint8_t)len);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
uint32_t gl_twi_write(uint8_t address, const uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI0_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_tx(&st_twi, address, buf, (uint8_t)len, false);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
uint32_t gl_twi_write_no_stop(uint8_t address, const uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI0_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_tx(&st_twi, address, buf, (uint8_t)len, true);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
/* TWI 1 */
#if TWI1_ENABLED==1
#define INTERNAL_TWI1_ENABLED
#define ST_TWI1_INSTANCE 1
static const nrf_drv_twi_t st_twi1 = NRF_DRV_TWI_INSTANCE(ST_TWI1_INSTANCE);
#endif
static bool st_twi1_initialized = false;
void gl_twi1_init(uint8_t sda_pin, uint8_t scl_pin)
{
#ifdef INTERNAL_TWI1_ENABLED
    if (st_twi1_initialized)
        return;
    else
        st_twi1_initialized = st_twim_init(&st_twi1, sda_pin, scl_pin);
#endif
}
uint8_t gl_twi1_read_reg_byte(uint8_t address, uint8_t reg)
{
    uint8_t ret = 0;
#ifdef INTERNAL_TWI1_ENABLED
    if (fix_drv_twi_tx(&st_twi1, address, &reg, 1, true) == NRF_SUCCESS)
        fix_drv_twi_rx(&st_twi1, address, &ret, 1);
#endif
    return ret;
}
void gl_twi1_write_reg_byte(uint8_t address, uint8_t reg, uint8_t value)
{
#ifdef INTERNAL_TWI1_ENABLED
    uint8_t data[2] = { reg, value };
    fix_drv_twi_tx(&st_twi1, address, data, 2, false);
#endif
}
uint32_t gl_twi1_read_reg(uint8_t address, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_TWI1_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    ret = fix_drv_twi_tx(&st_twi1, address, &reg, 1, true);
    if (ret == NRF_SUCCESS)
        ret = fix_drv_twi_rx(&st_twi1, address, buf, (uint8_t)len);
#endif
    return ret;
}
#define TWI1_STAT_BUF_SIZE 32
#ifdef INTERNAL_TWI1_ENABLED
static uint8_t st_twi1_buf[TWI1_STAT_BUF_SIZE + 1];
#endif
uint32_t gl_twi1_write_reg(uint8_t address, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint32_t ret = NRF_ERROR_NOT_SUPPORTED;
#ifdef INTERNAL_TWI1_ENABLED
    int i;
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    if (len <= TWI1_STAT_BUF_SIZE)
    {
        st_twi1_buf[0] = reg;
        for (i = 0; i < len; i++)
            st_twi1_buf[i + 1] = buf[i];
        ret = fix_drv_twi_tx(&st_twi1, address, st_twi1_buf, (uint8_t)(len + 1), false);
    }
    else
    {
        ret = fix_drv_twi_tx(&st_twi1, address, &reg, 1, true);
        if (ret == NRF_SUCCESS)
            ret = fix_drv_twi_tx(&st_twi1, address, buf, (uint8_t)len, false);
    }
#endif
    return ret;
}
int gl_twi1_is_present(uint8_t address)
{
#ifdef INTERNAL_TWI1_ENABLED
    return fix_drv_twi_tx(&st_twi1, address, 0, 0, false) ? 0 : 1;
#else
    return 0;
#endif
}
uint32_t gl_twi1_read(uint8_t address, uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI1_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_rx(&st_twi1, address, buf, (uint8_t)len);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
uint32_t gl_twi1_write(uint8_t address, const uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI1_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_tx(&st_twi1, address, buf, (uint8_t)len, false);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
uint32_t gl_twi1_write_no_stop(uint8_t address, const uint8_t *buf, uint32_t len)
{
#ifdef INTERNAL_TWI1_ENABLED
    if ((!buf) && len)
        return NRF_ERROR_NULL;
    return fix_drv_twi_tx(&st_twi1, address, buf, (uint8_t)len, true);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
/**/
