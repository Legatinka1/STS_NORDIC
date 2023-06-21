#include "GL_ROM.h"

#include "nrf_soc.h"
#include "nrf_fstorage.h"

#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#else
#include "nrf_fstorage_nvmc.h"
#endif

static int fstorage_error = 0;
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
        fstorage_error = 1;
}
static NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage);
static int fstorage_initialized = 0;
static int fstorage_init(void)
{
    if (!fstorage_initialized)
    {
        uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
        uint32_t const page_sz = NRF_FICR->CODEPAGESIZE;
        uint32_t const code_sz = NRF_FICR->CODESIZE;
        uint32_t end_addr = (bootloader_addr != 0xFFFFFFFF ? bootloader_addr : (code_sz * page_sz));
        ret_code_t rc;
        nrf_fstorage_api_t * p_fs_api;
#ifdef SOFTDEVICE_PRESENT
        p_fs_api = &nrf_fstorage_sd;
#else
        p_fs_api = &nrf_fstorage_nvmc;
#endif
        fstorage.evt_handler = fstorage_evt_handler;
        fstorage.start_addr = end_addr - page_sz;
        fstorage.end_addr = end_addr - 1;
        rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
        fstorage_initialized = rc ? 0 : 1;
    }
    return fstorage_initialized;
}
static void fstorage_wait_for_ready(void)
{
    while (nrf_fstorage_is_busy(&fstorage))
#ifdef SOFTDEVICE_PRESENT
        // NOTE: fstorage_is_busy will always return busy if called from BLE event
        (void)sd_app_evt_wait();
#else
        __WFE();
#endif
}
static int fstrorage_read(uint32_t addr, void *p_dest, uint32_t len)
{
    if (!fstorage_init())
        return 0;
    fstorage_error = 0;
    if (nrf_fstorage_read(&fstorage, fstorage.start_addr + addr, p_dest, len))
        return 0;
    return fstorage_error ? 0 : 1;
}
static int fstrorage_write(uint32_t addr, const void *p_src, uint32_t len)
{
    if (!fstorage_init())
        return 0;
    fstorage_error = 0;
    if (nrf_fstorage_write(&fstorage, fstorage.start_addr + addr, p_src, len, NULL))
        return 0;
    fstorage_wait_for_ready();
    return fstorage_error ? 0 : 1;
}
static int fstrorage_erase(void)
{
    if (!fstorage_init())
        return 0;
    fstorage_error = 0;
    if (nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL))
        return 0;
    fstorage_wait_for_ready();
    return fstorage_error ? 0 : 1;
}

int gl_rom_is_available(void)
{
    return fstorage_init();
}
int gl_rom_read(unsigned int addr, void *data, unsigned int len)
{
    return fstrorage_read(addr, data, len);
}
int gl_rom_write(unsigned int addr, const void *data, unsigned int len)
{
    return fstrorage_write(addr, data, len);
}
int gl_rom_erase(void)
{
    return fstrorage_erase();
}
