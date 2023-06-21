#include "GL_SDC.h"
#include <stdint.h>
#include <stdbool.h>
#include <nrfx.h>

/* SDC */
/*
// See
// https://devzone.nordicsemi.com/f/nordic-q-a/43233/sd-card-fatfs-nrf52832---can-t-initialize-if-first-initialization-fails
// to fix files diskio_blkdev.c and nrfx_spim.c
// Also change in nrfx_spim.c: [#include "prs/nrfx_prs.h"] to [#include <drivers/src/prs/nrfx_prs.h>]
//
// May need to configure MISO pullup in sdk_config.h
//
// <0=> NRF_GPIO_PIN_NOPULL 
// <1=> NRF_GPIO_PIN_PULLDOWN 
// <3=> NRF_GPIO_PIN_PULLUP 
#define NRFX_SPIM_MISO_PULL_CFG 3 // 1
#define NRFX_SPI_MISO_PULL_CFG 3 // 1
#define NRF_SPI_DRV_MISO_PULLUP_CFG 3 // 1
*/
//#if (APP_SDCARD_ENABLED==1 && APP_SDCARD_SPI_INSTANCE==2)
#if APP_SDCARD_ENABLED==1
#define INTERNAL_APP_SDCARD_ENABLED
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_drv_spi.h"
static nrf_block_dev_sdc_work_t st_block_dev_sdc_work;
static nrf_block_dev_sdc_t st_block_dev_sdc =
    {
        .block_dev = { .p_ops = &nrf_block_device_sdc_ops },
        .info_strings = { .p_vendor = "Nordic", .p_product = "SDC", .p_revision = "1.00" },
        .sdc_bdev_config = { .block_size = SDC_SECTOR_SIZE, .sdc_config =
            APP_SDCARD_CONFIG(NRF_DRV_SPI_PIN_NOT_USED, NRF_DRV_SPI_PIN_NOT_USED, NRF_DRV_SPI_PIN_NOT_USED, NRF_DRV_SPI_PIN_NOT_USED) },
        .p_work = &st_block_dev_sdc_work
    };
/*
NRF_BLOCK_DEV_SDC_DEFINE(
        st_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);
*/
static diskio_blkdev_t st_drives[] =
{
    DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(st_block_dev_sdc, block_dev), NULL)
};
static void st_diskio_blkdev_default_wait_func(void)
{
    __WFE();
}
#endif
void gl_sdc_set_spi_pins(unsigned char mosi_pin, unsigned char miso_pin, unsigned char sck_pin, unsigned char cs_pin)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    st_block_dev_sdc.sdc_bdev_config.sdc_config.mosi_pin = mosi_pin;
    st_block_dev_sdc.sdc_bdev_config.sdc_config.miso_pin = miso_pin;
    st_block_dev_sdc.sdc_bdev_config.sdc_config.sck_pin = sck_pin;
    st_block_dev_sdc.sdc_bdev_config.sdc_config.cs_pin = cs_pin;
#endif
}
void gl_sdc_set_wait_func(void (*wait_func)(void))
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    st_drives[0].config.wait_func = wait_func;
#endif
}
static bool st_sdc_is_registered = false;
unsigned char gl_sdc_init(void)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    DSTATUS disk_state = STA_NOINIT;
    uint32_t retries = 3;
    void (*wait_func)(void) = NULL;
    if (!st_sdc_is_registered)
    {
        diskio_blockdev_register(st_drives, 1/*(sizeof(st_drives) / sizeof((st_drives)[0]))*/);
        st_sdc_is_registered = true;
    }
    wait_func = st_drives[0].config.wait_func;
    st_drives[0].config.wait_func = st_diskio_blkdev_default_wait_func;
    while (retries && disk_state)
    {
        disk_state = disk_initialize(0);
        --retries;
    }
    st_drives[0].config.wait_func = wait_func;
    return disk_state;
#else
    return 0x01;
#endif
}
unsigned char gl_sdc_status(void)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    return disk_status(0);
#else
    return 0x01;
#endif
}
unsigned long gl_sdc_capacity_in_megabytes(void)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    uint32_t blocks_per_mb = (1024uL * 1024uL) / st_block_dev_sdc.block_dev.p_ops->geometry(&st_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = st_block_dev_sdc.block_dev.p_ops->geometry(&st_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    return capacity;
#else
    return 0;
#endif
}
unsigned char gl_sdc_uninit(void)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    if (!st_sdc_is_registered)
        return STA_NOINIT;
    return disk_uninitialize(0);
#else
    return 0x01;
#endif
}
unsigned char gl_sdc_reinit(void)
{
#ifdef INTERNAL_APP_SDCARD_ENABLED
    if (!st_sdc_is_registered)
        return STA_NOINIT;
    gl_sdc_uninit();
    return gl_sdc_init();
#else
    return 0x01;
#endif
}
/**/
