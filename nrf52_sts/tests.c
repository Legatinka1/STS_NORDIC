#include "tests.h"

#include "common.h"

/* Test */

#include "nrf_gpio.h"
#include "nrf_delay.h"

static void st_twi_presence_test(void)
{
    uint8_t address;
    uint32_t result;
    d_print("I2C DEVICES FOUND:");
    for (address = 0x03; address <= 0x77; address++)
    {
        /*
        if (((address >= 0x30) && (address <= 0x37)) || ((address >= 0x50) && (address <= 0x5F)))
            result = gl_twi_read(address, 0, 0);
        else
            result = gl_twi_write(address, 0, 0);
        if (!result)
            d_printf(" %02X", address);
        */
        if (gl_twi_is_present(address))
            d_printf(" %02X", address);
    }
    d_print("\r\n");
}

#include "storage.h"

static void st_initial_test(int enable_wom_test)
{
    // I2C
    {
        //gl_twi_init(GEN_SDA_PIN, GEN_SCL_PIN);
        //st_twi_presence_test();
    }
    // GPIO
    {
        //nrf_gpio_cfg_output(LED_OR_PIN);
        //nrf_gpio_pin_set(LED_OR_PIN);
        //nrf_delay_ms(500);
        //nrf_gpio_pin_clear(LED_OR_PIN);
        //nrf_delay_ms(500);
        //nrf_gpio_pin_set(LED_OR_PIN);
    }
}

static void st_continuous_test(int is_not_recording)
{
    /*
    {
        int n = d_getchar();
        while (n != -1)
        {
            d_prin(&n, 1);
            n = d_getchar();
        }
    }
    */
}

/* Interface */

int test_in_setup(int param)
{
    st_initial_test(param);
    return 0;
}

int test_in_loop(int param)
{
    st_continuous_test(param);
    return 0;
}
