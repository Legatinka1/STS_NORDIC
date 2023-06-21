#include "ctrl.h"


#define CTRL_FIRMWARE_VERSION_MAJOR 1 // 4 bytes, TYPE(mag).Major.Minor.Build
#define CTRL_FIRMWARE_VERSION_MINOR 20 // 4 bytes, TYPE(mag).Major.Minor.Build
#define CTRL_FIRMWARE_VERSION_BUILD 1 // 4 bytes, TYPE(mag).Major.Minor.Build

#include "common.h"
#include "cap.h"

#include <string.h>
#include <stdlib.h>
#include "nrf_delay.h"

//// BLE Configuration

#define NAME_PREFIX "STS"

void ctrl_define_ble_behavior(
    const unsigned char mac_address[6],
    char device_name[32],
    unsigned long *adv_interval,
    unsigned long *adv_duration,
    unsigned short *adv_data_service_uuid,
    unsigned short *adv_scan_response_company_identifier,
    int *adv_scan_response_service_uuid_exposed,
    int *adv_initially_active,
    int *adv_restart_on_stop,
    int *adv_hibernate_on_stop,
    unsigned short *conn_interval_min,
    unsigned short *conn_interval_max,
    int *rssi_monitoring_allowed
    )
{
    sprintf(device_name, NAME_PREFIX "_%02X%02X", mac_address[1], mac_address[0]);
    *adv_interval = 400; // 400 is 250 ms (unit is 0.625 ms)
    *adv_duration = 1000; // 1000 is 10 s (unit is 100 ms, maximum value is 18000 that is 180 s)
    *adv_data_service_uuid = 0x180F; // Not present is 0, Device Information is 0x180A, Battery is 0x180F
    *adv_scan_response_company_identifier = 0; // Not present is 0, Nordic is 0x0059, Apple is 0x004C - manufacturer specific data in scan response packet (beacon)
    *adv_scan_response_service_uuid_exposed = 0; // 0 means false, 1 means true; Service 128-bit identifier in scan response packet; can't coexist with Company Identifier
    *adv_initially_active = 1; // 0 means false, 1 means true
    *adv_restart_on_stop = 1; // 0 means false, 1 means true
    *adv_hibernate_on_stop = 0; // 0 means false, 1 means true; not relevant if Restart on Stop is true
    *conn_interval_min = 8; // 8 is 10 ms (unit is 1.25 ms)
    *conn_interval_max = 60; // 60 is 75 ms (unit is 1.25 ms)
    *rssi_monitoring_allowed = 0; // 0 means false, 1 means true
}

//// Advertisement

static unsigned char st_battery_decivolts = 0;
static unsigned char st_battery_level = 100;
static unsigned char st_cpu_voltage_decivolts = 0;
static unsigned char st_cpu_voltage_level = 100;
static unsigned char st_cpu_temperature = 25;
static unsigned short st_report_counter = 0;

void ctrl_fill_adv_name(char *name, unsigned short len)
{
}

void ctrl_fill_adv_data(unsigned char *data, unsigned short len)
{
    unsigned short counter;
    if (len < 4)
        return;
    data[0] = st_battery_level;
    data[1] = st_cpu_temperature;
    counter = st_report_counter;
    data[2] = (unsigned char)(counter & 0x00FF);
    data[3] = (unsigned char)((counter >> 8) & 0x00FF);
}

void ctrl_fill_adv_man(unsigned char *data, unsigned short len)
{
}

//// Security

#include "peer_manager.h"
#include "peer_manager_handler.h"

static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    cap_preinit_peer_manager();
    pm_init();
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
    sec_param.min_key_size = 7;
    sec_param.max_key_size = 16;
    pm_sec_params_set(&sec_param);
    pm_register(pm_evt_handler);
}

//// Pinout

#define ADS8513_DATA_PIN    16 // DATA
#define ADS8513_DATACLK_PIN 17 // DATACLK
#define ADS8513_CS_PIN      14 // CS
#define ADS8513_CONV_PIN    20 // CONV
#define ADS8513_BUSY_PIN    19 // BUSY

#define LED_R1_PIN          21
#define LED_Y1_PIN          23
#define LED_G1_PIN          24
#define LED_R2_PIN          25
#define LED_G2_PIN          26
#define LED_Y2_PIN          27

#define DISABLE_CHG_PIN      8 // Output
#define CHG_PIN              9 // Input
#define EOC_PIN             10 // Input
#define SOLENOID_PIN        11 // Output
#define SUP_CLEAR_PIN       12 // Output

#define AIN_VBAT_INPUT       NRF_SAADC_INPUT_AIN0

//// Setup

static void st_init_gpio(void)
{
    nrf_gpio_pin_set(LED_R1_PIN);
    nrf_gpio_cfg_output(LED_R1_PIN);
    nrf_gpio_pin_set(LED_Y1_PIN);
    nrf_gpio_cfg_output(LED_Y1_PIN);
    nrf_gpio_pin_set(LED_G1_PIN);
    nrf_gpio_cfg_output(LED_G1_PIN);
    nrf_gpio_pin_set(LED_R2_PIN);
    nrf_gpio_cfg_output(LED_R2_PIN);
    nrf_gpio_pin_set(LED_G2_PIN);
    nrf_gpio_cfg_output(LED_G2_PIN);
    nrf_gpio_pin_set(LED_Y2_PIN);
    nrf_gpio_cfg_output(LED_Y2_PIN);
    nrf_gpio_pin_clear(DISABLE_CHG_PIN);
    nrf_gpio_cfg_output(DISABLE_CHG_PIN);
    nrf_gpio_cfg_input(CHG_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(EOC_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_clear(SOLENOID_PIN);
    nrf_gpio_cfg_output(SOLENOID_PIN);
    nrf_gpio_pin_clear(SUP_CLEAR_PIN);
    nrf_gpio_cfg_output(SUP_CLEAR_PIN);
}

//static void st_sdc_wait_func(void) { __WFE(); }

static void st_init_interfaces(void)
{
    // COM
    //gl_com_init(GEN_TX_PIN, GEN_RX_PIN, GEN_BAUDRATE, 0/*1*/);
    // RTC
    gl_rtc_config();
    // SDC
    //gl_sdc_set_spi_pins(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN);
    //gl_sdc_set_wait_func(st_sdc_wait_func);
    //gl_sdc_init();
    // SPI
    //gl_spi_prepare_ss_pin(GEN_CS0_PIN);
    //gl_spi_init(GEN_MOSI_PIN, GEN_MISO_PIN, GEN_SCK_PIN, 0);
    // TWI
    //gl_twi_init(GEN_SDA_PIN, GEN_SCL_PIN);
    // ADC
    gl_adc_read(GL_ADC_PIN_VDD);
    // ROM
    //gl_rom_is_available();
    // USB
    gl_usb_init();
    // PWM
    //gl_pwm_set_pins(1, BUZZERS_PWM1_PIN, BUZZERS_PWM2_PIN, GL_PWM_PIN_DISCONNECTED, GL_PWM_PIN_DISCONNECTED);
    // I2S
    //gl_i2s_init(GL_I2S_PIN_DISCONNECTED, I2S_BCK_PIN, I2S_WS_PIN, I2S_DATAI_PIN);
}

static void st_init_devices(void)
{
}

static void st_init_data(void);

//static unsigned short st_timer_period = 100;
static unsigned short st_timer_frequency = 10;
static unsigned int st_timer_watchdog_timeout = 5000; // Set five seconds (5000 ms) delay to watchdog; 0 means disabled (it's disabled by default)

void ctrl_init(void)
{
    peer_manager_init();
    //
    use_debug_output = USE_DEBUG_OUTPUT_RTT;
    st_init_gpio();
    st_init_interfaces();
    st_init_devices();
    st_init_data();
    st_report_counter = 0;
}

void ctrl_begin(void)
{
    //periodical_timer_set_period(st_timer_period);
    periodical_timer_set_frequency(st_timer_frequency);
    periodical_timer_sleep();
    periodical_timer_set_watchdog_timeout(st_timer_watchdog_timeout);
}

//// Internals

#define ST_BATTERY_IS_RECHARGEABLE 1 // 1 means rechargeable, 0 means not rechargeable

static void st_chip_get_battery_level(void)
{
    int32_t v, dv;
    // CPU
    v = gl_adc_read(GL_ADC_PIN_VDD);
    dv = GL_ADC_VALUE_TO_DECIVOLTS(v);
    st_cpu_voltage_decivolts = (unsigned char)dv;
    st_cpu_voltage_level = adc_convert_decivolts_to_battery_level(dv, ST_BATTERY_IS_RECHARGEABLE);
    // USER
#ifdef AIN_VBAT_INPUT
    v = gl_adc_read(AIN_VBAT_INPUT);
#if defined(AIN_VBAT_R_UP) && defined(AIN_VBAT_R_DOWN)
    v = GL_ADC_VALUE_DIVIDE(v, AIN_VBAT_R_UP, AIN_VBAT_R_DOWN);
#else
    v *= 2;
#endif
    dv = GL_ADC_VALUE_TO_DECIVOLTS(v);
    st_battery_decivolts = (unsigned char)dv;
    st_battery_level = adc_convert_decivolts_to_battery_level(dv, ST_BATTERY_IS_RECHARGEABLE);
#else
    st_battery_decivolts = st_cpu_voltage_decivolts;
    st_battery_level = st_cpu_voltage_level;
#endif
}

static void st_chip_get_temperature(void)
{
    int32_t temp;
    temp = gl_temperature_read();
    temp = GL_TEMPERATURE_TO_CELSIUS_DEGREES(temp);
    if (temp > 255) temp = 255; else
    if (temp < 0) temp = 0;
    st_cpu_temperature = (unsigned char)temp;
}

//// BLE FIFO

#include "app_fifo.h"

static uint32_t st_app_fifo_used(app_fifo_t * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

static uint32_t st_app_fifo_free(app_fifo_t * p_fifo)
{
    return (p_fifo->buf_size_mask + 1) - st_app_fifo_used(p_fifo);
}

static unsigned char st_ble_fifo_buf_rx[512];
static app_fifo_t st_ble_fifo_rx = { .p_buf = st_ble_fifo_buf_rx, .buf_size_mask = sizeof(st_ble_fifo_buf_rx) - 1, .read_pos = 0, .write_pos = 0 };

static unsigned short st_ble_buf_store(const void *data, unsigned short len)
{
    uint32_t written = len;
    if (!data)
        return 0;
    if (app_fifo_write(&st_ble_fifo_rx, (uint8_t *)data, &written) == NRF_SUCCESS)
        return written;
    return 0;
}

static unsigned short st_ble_buf_get_readable_count(void)
{
    return (unsigned short)st_app_fifo_used(&st_ble_fifo_rx);
}

static unsigned short st_ble_buf_read(void *data, unsigned short count, int peek)
{
    unsigned short ret;
    uint32_t old_read_pos = st_ble_fifo_rx.read_pos, real_count = count;
    ret = app_fifo_read(&st_ble_fifo_rx, (uint8_t *)data, &real_count) == NRF_SUCCESS ? (unsigned short)real_count : 0;
    if (peek)
        st_ble_fifo_rx.read_pos = old_read_pos;
    return ret;
}

static void st_ble_buf_flush(void)
{
    app_fifo_flush(&st_ble_fifo_rx);
}

//// Serial Buffer

#define SERBUF_VAL_SYNC 0xEF

#define SERBUF_POS_SYNC 0
#define SERBUF_POS_LENGTH 1 // Length of payload minus one (0..255 means 1..256)
#define SERBUF_POS_HASH 2 // Hash of opcode and payload
#define SERBUF_POS_OPCODE 3
#define SERBUF_POS_PAYLOAD 4 // 1 to 256 bytes

#define SERBUF_MSGLEN_MIN 5
#define SERBUF_MSGLEN_MAX 260

typedef struct
{
    unsigned short count;
    unsigned char data[SERBUF_MSGLEN_MAX];
} serbuf_collector_t;

void serbuf_collector_reset(serbuf_collector_t *sc)
{
    if (!sc)
        return;
    sc->count = 0;
}

void serbuf_collector_append(serbuf_collector_t *sc, unsigned char b)
{
    if (!sc)
        return;
    if (!sc->count)
    {
        if (b == SERBUF_VAL_SYNC)
        {
            sc->data[0] = b;
            sc-> count = 1;
        }
        return;
    }
    if (sc->count < SERBUF_MSGLEN_MAX)
    {
        sc->data[sc->count] = b;
        ++sc->count;
        return;
    }
    {
        unsigned char *dest = sc->data, *src = sc->data + 1;
        unsigned short rest = SERBUF_MSGLEN_MAX - 1;
        while (rest ? ((*src) != SERBUF_VAL_SYNC) : 0)
        {
            --rest;
            ++src;
        }
        if (!rest)
        {
            if (b != SERBUF_VAL_SYNC)
            {
                sc->count = 0;
                return;
            }
            sc->data[0] = b;
            sc-> count = 1;
            return;
        }
        sc->count = rest + 1;
        while (rest--)
            *dest++ = *src++;
        sc->data[sc->count - 1] = b;
    }
}

int serbuf_collector_retrieve(serbuf_collector_t *sc, unsigned char *opcode, void *payload, unsigned short *payload_length)
{
    if (payload_length)
        *payload_length = 0;
    if (opcode)
        *opcode = 0;
    if (!(sc && opcode && payload && payload_length))
        return 0;
    if (sc->count < SERBUF_MSGLEN_MIN)
        return 0;
    if ((sc->count - SERBUF_MSGLEN_MIN) < sc->data[SERBUF_POS_LENGTH])
        return 0;
    {
        unsigned short len = 2 + sc->data[SERBUF_POS_LENGTH];
        unsigned char chk = 0, *src = sc->data + SERBUF_POS_OPCODE, *dest;
        while (len--)
            chk += (chk << 1) + *src++;
        if (chk == sc->data[SERBUF_POS_HASH])
        {
            *opcode = sc->data[SERBUF_POS_OPCODE];
            src = sc->data + SERBUF_POS_PAYLOAD;
            dest = (unsigned char *)payload;
            len = 1 + sc->data[SERBUF_POS_LENGTH];
            *payload_length = len;
            while (len--)
                *dest++ = *src++;
            len = SERBUF_MSGLEN_MIN + sc->data[SERBUF_POS_LENGTH];
            if (sc->count == len)
            {
                sc->count = 0;
                return *payload_length;
            }
            dest = sc->data;
            src = sc->data + len;
            sc->count -= len;
            len = sc->count;
            while (len--)
                *dest++ = *src++;
            return *payload_length;
        }
        {
            unsigned short rest = sc->count - 1;
            src = sc->data + 1;
            while (rest && ((*src) != SERBUF_VAL_SYNC))
            {
                --rest;
                ++src;
            }
            if (!rest)
            {
                sc-> count = 0;
                return 0;
            }
            dest = sc->data;
            sc->count = rest;
            while (rest--)
                *dest++ = *src++;
            return -1;
        }
    }
}

int serbuf_message_compose(unsigned char opcode, const void *payload, unsigned short payload_length, unsigned char *message, unsigned short *message_length)
{
    if (message_length)
        *message_length = 0;
    else
        return 0;
    if (!(payload && payload_length && (payload_length <= 256) && message))
        return 0;
    {
        unsigned char chk = opcode, *src = (unsigned char *)payload, *dest = message + SERBUF_POS_PAYLOAD;
        unsigned short len = payload_length;
        *message_length = SERBUF_POS_PAYLOAD + payload_length;
        message[SERBUF_POS_SYNC] = SERBUF_VAL_SYNC;
        message[SERBUF_POS_LENGTH] = (unsigned char)(payload_length - 1);
        message[SERBUF_POS_OPCODE] = opcode;
        while (len--)
        {
            chk += (chk << 1) + *src;
            *dest++ = *src++;
        }
        message[SERBUF_POS_HASH] = chk;
        return *message_length;
    }
}

//// Serial Conversation

#define ser_read(d, l) gl_usb_read(d, l, 0)
#define ser_write(d, l) gl_usb_write(d, l, 0)

static serbuf_collector_t st_serbuf_collector = { 0 };

static int serbuf_collected_from_ble = 0;
static serbuf_collector_t st_serbuf_collector_ble = { 0 };

int ser_command_receive(unsigned char *opcode, void *payload, unsigned short *payload_length)
{
    int ret;
    unsigned char b;
    if (cap_is_connected())
    {
        do
        {
            ret = serbuf_collector_retrieve(&st_serbuf_collector_ble, opcode, payload, payload_length);
            if (ret > 0)
            {
                serbuf_collected_from_ble = 1;
                return ret;
            }
        }
        while (ret);
        while (st_ble_buf_read(&b, 1, 0) == 1)
        {
            serbuf_collected_from_ble = 1;
            serbuf_collector_append(&st_serbuf_collector_ble, b);
            do
            {
                ret = serbuf_collector_retrieve(&st_serbuf_collector_ble, opcode, payload, payload_length);
                if (ret > 0)
                {
                    serbuf_collected_from_ble = 1;
                    return ret;
                }
            }
            while (ret);
        }
    }
    else
    {
        st_ble_buf_flush();
        serbuf_collector_reset(&st_serbuf_collector_ble);
    }
    do
    {
        ret = serbuf_collector_retrieve(&st_serbuf_collector, opcode, payload, payload_length);
        if (ret > 0)
        {
            serbuf_collected_from_ble = 0;
            return ret;
        }
    }
    while (ret);
    while (ser_read(&b, 1) == 1)
    {
        serbuf_collector_append(&st_serbuf_collector, b);
        do
        {
            ret = serbuf_collector_retrieve(&st_serbuf_collector, opcode, payload, payload_length);
            if (ret > 0)
            {
                serbuf_collected_from_ble = 0;
                return ret;
            }
        }
        while (ret);
    }
    return 0;
}

int ser_command_send(unsigned char opcode, const void *payload, unsigned short payload_length)
{
    serbuf_collector_t sc;
    periodical_timer_watchdog_feed();
    if (serbuf_message_compose(opcode, payload, payload_length, sc.data, &sc.count) > 0)
        if (serbuf_collected_from_ble)
            return cap_send(sc.data, sc.count, 0) ? sc.count : 0;
        else
            return ser_write(sc.data, sc.count) ? sc.count : 0;
    return 0;
}

//// ADS8513

static short ads8513_read(void)
{
    static int ads8513_initialized = 0;
    short data = 0;
    int i;
    if (!ads8513_initialized)
    {
        ads8513_initialized = 1;
        nrf_gpio_pin_set(ADS8513_CONV_PIN);
        nrf_gpio_cfg_output(ADS8513_CONV_PIN);
        nrf_gpio_cfg_input(ADS8513_BUSY_PIN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_input(ADS8513_DATA_PIN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_pin_clear(ADS8513_DATACLK_PIN);
        nrf_gpio_cfg_output(ADS8513_DATACLK_PIN);
#if defined(ADS8513_CS_PIN)
        if (ADS8513_CS_PIN < NUMBER_OF_PINS)
        {
            nrf_gpio_pin_clear(ADS8513_CS_PIN);
            nrf_gpio_cfg_output(ADS8513_CS_PIN);
        }
#endif
#if defined(ADS8513_EXTINT_PIN)
        if (ADS8513_EXTINT_PIN < NUMBER_OF_PINS)
        {
            nrf_gpio_pin_clear(ADS8513_EXTINT_PIN);
            nrf_gpio_cfg_output(ADS8513_EXTINT_PIN);
        }
#endif
    }
    nrf_gpio_pin_clear(ADS8513_CONV_PIN);
    while (!nrf_gpio_pin_read(ADS8513_BUSY_PIN))
        ;
    for (i = 0; i < 16; i++)
    {
        nrf_gpio_pin_set(ADS8513_DATACLK_PIN);
        __NOP();
        data <<= 1;
        nrf_gpio_pin_clear(ADS8513_DATACLK_PIN);
        __NOP();
        if (nrf_gpio_pin_read(ADS8513_DATA_PIN))
            data |= 1;
    }
    nrf_gpio_pin_set(ADS8513_CONV_PIN);
    return data;
}

//// Shutter

static void st_shutter_close(void)
{
    nrf_gpio_pin_clear(SOLENOID_PIN);
}

static void st_shutter_open(void)
{
    nrf_gpio_pin_set(SOLENOID_PIN);
}

//// Charger

#define ST_BATTERY_LOW_LEVEL_DECIVOLTS 28
#define ST_BATTERY_WARNING_LEVEL_DECIVOLTS 35
#define ST_BATTERY_HIGH_LEVEL_DECIVOLTS 42

static void st_charger_monitor(void)
{
    if (st_battery_decivolts <= ST_BATTERY_LOW_LEVEL_DECIVOLTS)
        nrf_gpio_pin_set(SUP_CLEAR_PIN);
    else
        nrf_gpio_pin_clear(SUP_CLEAR_PIN);
    if (st_battery_decivolts >= ST_BATTERY_HIGH_LEVEL_DECIVOLTS)
        nrf_gpio_pin_set(DISABLE_CHG_PIN);
    else
        nrf_gpio_pin_clear(DISABLE_CHG_PIN);
}

uint64_t st_charging_last_time = 0;

static int st_is_charging(void)
{
    uint64_t now = gl_rtc_seconds();
    if (nrf_gpio_pin_read(CHG_PIN) == 0)
    {
        st_charging_last_time = now;
        return 1;
    }
    else
    {
        uint64_t passed = now - st_charging_last_time;
        if (passed > 2)
            return 0;
        else
            return 1;
    }
}

//// LEDs

#define LEDS_RYG_COUNT 2
static uint8_t leds_ryg_array[LEDS_RYG_COUNT * 3] = { LED_R1_PIN, LED_G1_PIN, LED_Y1_PIN, LED_R2_PIN, LED_G2_PIN, LED_Y2_PIN };

static void st_led_ryg_set(int led, int r, int y, int g)
{
    if ((led < 1) || (led > LEDS_RYG_COUNT))
        return;
    nrf_gpio_pin_write(leds_ryg_array[(led - 1) * 3], r ? 0 : 1);
    nrf_gpio_pin_write(leds_ryg_array[((led - 1) * 3) + 2], y ? 0 : 1);
    nrf_gpio_pin_write(leds_ryg_array[((led - 1) * 3) + 1], g ? 0 : 1);
}

//// Average

static uint64_t st_average_period_ticks = 4; // 32768 / 8000

void st_set_averaged_period_in_ticks(uint64_t ticks)
{
    st_average_period_ticks = ticks >> 3;
}

static short st_get_averaged_measurement(void)
{
    uint64_t next = gl_rtc_ticks() + st_average_period_ticks;
    int i;
    long value = 0;
    for (i = 1; i <= 8; i++)
    {
        value += ads8513_read();
        if (i < 8)
        {
            while (gl_rtc_ticks() < next)
                ;
            next += st_average_period_ticks;
        }
    }
    value >>= 3;
    return (short)value;
}

//// Decimation

#define DECIMATION_WINDOW_SIZE_MAX 500
#define DECIMATION_WINDOW_SIZE_DEFAULT 20
static short st_decimation_window_data[DECIMATION_WINDOW_SIZE_MAX];
static short st_decimation_window_size = DECIMATION_WINDOW_SIZE_DEFAULT;
static int st_decimated_value_index = 0;
static long st_decimated_values_sum = 0;

static void st_decimation_init(void)
{
    int i;
    short value;
    st_decimated_value_index = 0;
    st_decimated_values_sum = 0;
    for (i = 0; i < st_decimation_window_size; i++)
    {
        value = st_get_averaged_measurement(); // ads8513_read();
        st_decimated_values_sum += value;
        st_decimation_window_data[i] = value;
    }
}

static short st_get_decimated_measurement(short *raw_value)
{
    short old_value = st_decimation_window_data[st_decimated_value_index];
    short new_value = st_get_averaged_measurement(); // ads8513_read();
    long value;
    if (raw_value)
        *raw_value = new_value;
    if (st_decimation_window_size <= 1)
        return new_value;
    st_decimation_window_data[st_decimated_value_index] = new_value;
    ++st_decimated_value_index;
    if (st_decimated_value_index >= st_decimation_window_size)
        st_decimated_value_index = 0;
    st_decimated_values_sum -= old_value;
    st_decimated_values_sum += new_value;
    value = st_decimated_values_sum;
    if (value < 0)
        value -= st_decimation_window_size >> 1;
    else
        value += st_decimation_window_size >> 1;
    value /= st_decimation_window_size;
    return (short)value;
}

static short st_set_decimation_window_size(short count)
{
    if ((count == st_decimation_window_size) || (count <= 0))
        return st_decimation_window_size;
    if (count >= DECIMATION_WINDOW_SIZE_MAX)
        st_decimation_window_size = DECIMATION_WINDOW_SIZE_MAX;
    else
        st_decimation_window_size = count;
    st_decimation_init();
    return st_decimation_window_size;
}

//// Storage

typedef struct
{
    unsigned short magic;
    unsigned short length;
    unsigned long version;
    unsigned long factory_serial;
    unsigned long long_placeholder;
    float K;
    float B;
    float Zero;
    float C1;
    float D1;
    float C2;
    float D2;
    float float_placeholder[20];
    unsigned long CRC;
} sts_configuration_t;

static sts_configuration_t st_sts_configuration;

int st_load_configuration(void)
{
    return gl_rom_read(0, &st_sts_configuration, sizeof(st_sts_configuration));
}

int st_save_configuration(void)
{
    gl_rom_erase();
    return gl_rom_write(0, &st_sts_configuration, sizeof(st_sts_configuration));
}

//// Flow

#define OPCODE_SAMPLING 0
#define OPCODE_OUTPUT_CONTROL 1
#define OPCODE_DECIMATION_WINDOW_SIZE_UPDATE 8
#define OPCODE_CALIBRATED_ZERO_GET 9
#define OPCODE_SAMPLING_ALGO_START 10
#define OPCODE_SAMPLING_ALGO_START_WITHOUT_INHALE 11
#define OPCODE_SAMPLING_ALGO_INDICES_GET 20
#define OPCODE_VERSION 100
#define OPCODE_CONFIG_GET 110
#define OPCODE_CONFIG_SET 111
#define OPCODE_BATTERY_LEVEL 120
#define OPCODE_SWITCH_TO_DFU 127

#define SAMPLING_STATE_NONE  0
#define SAMPLING_STATE_FREE  1
#define SAMPLING_STATE_ALGO  2

static int st_sampling_state = 0;

#define SAMPLE_TYPE_FREE 0
#define SAMPLE_TYPE_PRERECORD 1
#define SAMPLE_TYPE_INHALE 2
#define SAMPLE_TYPE_EXHALE_PREMAX 3
#define SAMPLE_TYPE_EXHALE_POSTMAX 4
#define SAMPLE_TYPE_EXHALE_POSTMIN 5
#define SAMPLE_TYPE_END 6

#define SAMPLE_TYPE_PREREPORT_ADD 20

#define SAMPLING_FREQUENCY_MAX 10000
#define SAMPLING_FREQUENCY_ALGO 1000
#define SAMPLING_DURATION_ALGO_MAX 15

static int st_calibration_done = 0;
static unsigned short st_sampling_frequency_for_calibration = SAMPLING_FREQUENCY_ALGO;
static short st_sampling_calibrated_zero = 0;

static uint64_t st_sampling_period_ticks = 0;
static unsigned short st_sampling_frequency = SAMPLING_FREQUENCY_ALGO;
static unsigned short st_sampling_duration = SAMPLING_DURATION_ALGO_MAX;
static short st_sampling_threshold_inhale_start = -128;
static short st_sampling_threshold_exhale_start = 128;
static unsigned short st_sampling_threshold_trend_diff = 128;
static unsigned short st_sampling_threshold_exhale_min = 128;
static unsigned short st_sampling_threshold_silence_spread = 128;
static uint64_t st_sampling_period_ticks_fast = 0;
static uint64_t st_sampling_period_ticks_slow = 0;
static int st_sampling_without_inhale = 0;

#define SAMPLING_CHUNK_PREFIX_LEN 2
#define SAMPLING_CHUNK_LEN 28
#define SAMPLING_CHUNK_BUFFER_LEN (SAMPLING_CHUNK_PREFIX_LEN + SAMPLING_CHUNK_LEN)
#define SAMPLING_SAVED_BUFFER_LEN (SAMPLING_FREQUENCY_ALGO * SAMPLING_DURATION_ALGO_MAX)

static int16_t st_sampling_chunk_buffer[SAMPLING_CHUNK_BUFFER_LEN];
static short st_sampling_chunk_counter = 0;
static int16_t st_sampling_saved_buffer[SAMPLING_SAVED_BUFFER_LEN];
static int st_sampling_saved_buffer_index = 0;

static int st_sampling_saved_buffer_index_algo_exhale_start = SAMPLING_SAVED_BUFFER_LEN;
static int st_sampling_saved_buffer_index_algo_exhale_max = SAMPLING_SAVED_BUFFER_LEN;
static int st_sampling_saved_buffer_index_algo_exhale_min = SAMPLING_SAVED_BUFFER_LEN;
static short st_sampling_saved_buffer_value_algo_start = 0;
static short st_sampling_saved_buffer_value_algo_exhale_min = 32767;
static short st_sampling_saved_buffer_value_algo_exhale_max = -32768;

#define SAMPLING_SILENCE_COUNT_ENOUGH 100

static int st_sampling_silence_count = 0;

#define SAMPLING_CALIBRATION_QUICK_COUNT 100

static void st_sampling_calibrate_process(unsigned short frequency, unsigned short count)
{
    uint64_t freq = frequency ? frequency : (st_sampling_frequency ? st_sampling_frequency : SAMPLING_FREQUENCY_ALGO);
    int cnt = count ? count : freq;
    uint64_t period_ticks = (32768 + (freq >> 1)) / freq;
    uint64_t next = gl_rtc_ticks() + period_ticks;
    int i = 0;
    long sum = 0;
    if (count >= 1)
    {
        while (i < count)
        {
            sum += ads8513_read();
            ++i;
            if (i < count)
            {
                while (gl_rtc_ticks() < next)
                    ;
                next += period_ticks;
            }
        }
        if (sum < 0)
            sum -= count >> 1;
        else
            sum += count >> 1;
        sum /= count;
    }
    st_sampling_calibrated_zero = (short)sum;
    //
    //d_printf("Calibrated zero = %d\r\n", st_sampling_calibrated_zero);
    //
}

static void st_sampling_free_process(void)
{
    uint64_t next = gl_rtc_ticks() + st_sampling_period_ticks;
    int i, j;
    i = 0;
    st_sampling_chunk_buffer[0] = SAMPLE_TYPE_FREE;
    while (i < st_sampling_frequency)
    {
        st_sampling_chunk_buffer[1] = st_sampling_chunk_counter++;
        j = SAMPLING_CHUNK_PREFIX_LEN;
        while ((j < SAMPLING_CHUNK_BUFFER_LEN) && (i < st_sampling_frequency))
        {
            st_sampling_chunk_buffer[j] = ads8513_read();
            ++j;
            ++i;
            if ((j == SAMPLING_CHUNK_BUFFER_LEN) || (i == st_sampling_frequency))
                ser_command_send(OPCODE_SAMPLING, st_sampling_chunk_buffer, j << 1);
            if (i < st_sampling_frequency)
            {
                while (gl_rtc_ticks() < next)
                    ;
                next += st_sampling_period_ticks;
            }
        }
    }
}

static int st_sampling_algo_cycles_count = 0;
static int st_sampling_algo_cycles_index = 0;

static void st_sampling_algo_process_report(short sample_type, int first_index, int last_index)
{
    int i, j, last;
    i = first_index;
    last = last_index;
    st_sampling_chunk_buffer[0] = sample_type;
    while (i < last)
    {
        st_sampling_chunk_buffer[1] = st_sampling_chunk_counter++;
        j = SAMPLING_CHUNK_PREFIX_LEN;
        while ((j < SAMPLING_CHUNK_BUFFER_LEN) && (i < last))
        {
            st_sampling_chunk_buffer[j] = st_sampling_saved_buffer[i];
            ++j;
            ++i;
            if ((j == SAMPLING_CHUNK_BUFFER_LEN) || (i == last))
                ser_command_send(OPCODE_SAMPLING, st_sampling_chunk_buffer, j << 1);
        }
    }
}

static void st_sampling_algo_process(void)
{
    uint64_t next = gl_rtc_ticks();
    short value, value_decimated, value_raw, prereport;
    int i, last;
    // Before start
    if (st_sampling_algo_cycles_index == 0)
    {
        next += st_sampling_period_ticks;
        value_decimated = st_get_decimated_measurement(&value_raw);
        value = value_decimated - st_sampling_calibrated_zero;
        i = 0;
        if (st_sampling_without_inhale)
        {
            while ((value < st_sampling_threshold_exhale_start) && (i < st_sampling_frequency))
            {
                ++i;
                value_decimated = st_get_decimated_measurement(&value_raw);
                value = value_decimated - st_sampling_calibrated_zero;
                if (i < st_sampling_frequency)
                {
                    while (gl_rtc_ticks() < next)
                        ;
                    next += st_sampling_period_ticks;
                }
            }
            if (value >= st_sampling_threshold_exhale_start)
                st_sampling_saved_buffer_index_algo_exhale_start = 0;
        }
        else
        {
            while ((value > st_sampling_threshold_inhale_start) && (i < st_sampling_frequency))
            {
                ++i;
                value_decimated = st_get_decimated_measurement(&value_raw);
                value = value_decimated - st_sampling_calibrated_zero;
                if (i < st_sampling_frequency)
                {
                    while (gl_rtc_ticks() < next)
                        ;
                    next += st_sampling_period_ticks;
                }
            }
        }
        if (i >= st_sampling_frequency)
        {
            st_sampling_saved_buffer[0] = value_decimated;
            st_sampling_algo_process_report(SAMPLE_TYPE_PREREPORT_ADD + SAMPLE_TYPE_PRERECORD, 0, 1);
            return;
        }
        st_shutter_close();
        periodical_timer_set_frequency(1);
        next = gl_rtc_ticks();
        st_sampling_silence_count = 0;
    }
    // Define frequency
    ++st_sampling_algo_cycles_index;
    next += st_sampling_period_ticks;
    // Collect data
    i = 0;
    while ((i < st_sampling_frequency) && (st_sampling_silence_count < SAMPLING_SILENCE_COUNT_ENOUGH) && (st_sampling_saved_buffer_index < SAMPLING_SAVED_BUFFER_LEN))
    {
        // Register value
        value_decimated = st_get_decimated_measurement(&value_raw);
        value = value_decimated - st_sampling_calibrated_zero;
        st_sampling_saved_buffer[st_sampling_saved_buffer_index] = value_decimated;
        // Find exhale start
        if (st_sampling_saved_buffer_index < st_sampling_saved_buffer_index_algo_exhale_start)
            if (value >= st_sampling_threshold_exhale_start)
                st_sampling_saved_buffer_index_algo_exhale_start = st_sampling_saved_buffer_index;
            else
                ;
        // Find exhale max
        else
            if (st_sampling_saved_buffer_index < st_sampling_saved_buffer_index_algo_exhale_max)
                if (value > (st_sampling_saved_buffer_value_algo_exhale_max + st_sampling_threshold_trend_diff))
                    st_sampling_saved_buffer_value_algo_exhale_max = value;
                else
                    if ((value + st_sampling_threshold_trend_diff) < st_sampling_saved_buffer_value_algo_exhale_max)
                        st_sampling_saved_buffer_index_algo_exhale_max = st_sampling_saved_buffer_index;
                    else
                        ;
        // Find exhale min
            else
                if (st_sampling_saved_buffer_index < st_sampling_saved_buffer_index_algo_exhale_min)
                    if ((value + st_sampling_threshold_trend_diff) < st_sampling_saved_buffer_value_algo_exhale_min)
                    {
                        st_sampling_saved_buffer_value_algo_exhale_min = value;
                        if (st_sampling_saved_buffer_value_algo_exhale_min < st_sampling_threshold_exhale_min)
                        {
                            st_sampling_saved_buffer_index_algo_exhale_min = st_sampling_saved_buffer_index;
                            st_shutter_open();
                        }
                    }
                    else
                        ;
        // Find silence
                else
                    if ((value >= -st_sampling_threshold_silence_spread) && (value <= st_sampling_threshold_silence_spread))
                        ++st_sampling_silence_count;
                    else
                        st_sampling_silence_count = 0;
        // Increment sample index
        ++st_sampling_saved_buffer_index;
        // Delay
        ++i;
        if ((i < st_sampling_frequency) && (st_sampling_silence_count < SAMPLING_SILENCE_COUNT_ENOUGH))
        {
            while (gl_rtc_ticks() < next)
                ;
            next += st_sampling_period_ticks;
        }
    }
    // Pre-report
    if ((st_sampling_algo_cycles_index < st_sampling_algo_cycles_count) && (st_sampling_silence_count < SAMPLING_SILENCE_COUNT_ENOUGH))
    {
        if (st_sampling_saved_buffer_index > st_sampling_saved_buffer_index_algo_exhale_min)
            prereport = SAMPLE_TYPE_EXHALE_POSTMIN;
        else
            if (st_sampling_saved_buffer_index > st_sampling_saved_buffer_index_algo_exhale_max)
                prereport = SAMPLE_TYPE_EXHALE_POSTMAX;
            else
                if (st_sampling_saved_buffer_index > st_sampling_saved_buffer_index_algo_exhale_start)
                    prereport = SAMPLE_TYPE_EXHALE_PREMAX;
                else
                    prereport = SAMPLE_TYPE_INHALE;
        st_sampling_algo_process_report(SAMPLE_TYPE_PREREPORT_ADD + prereport, st_sampling_saved_buffer_index - 1, st_sampling_saved_buffer_index);
    }
    else
    // Full report at the end
    {
        st_sampling_chunk_counter = 0;
        value = st_sampling_saved_buffer[0];
        st_sampling_saved_buffer[0] = st_sampling_calibrated_zero;
        st_sampling_algo_process_report(SAMPLE_TYPE_PRERECORD, 0, 1);
        st_sampling_saved_buffer[0] = value;
        st_shutter_close();
        st_sampling_chunk_counter = 0;
        i = 0;
        // Inhale
        last = st_sampling_saved_buffer_index_algo_exhale_start < st_sampling_saved_buffer_index ? st_sampling_saved_buffer_index_algo_exhale_start : st_sampling_saved_buffer_index;
        st_sampling_algo_process_report(SAMPLE_TYPE_INHALE, i, last);
        i = last;
        // Pre-exhale-max samples
        last = st_sampling_saved_buffer_index_algo_exhale_max < st_sampling_saved_buffer_index ? st_sampling_saved_buffer_index_algo_exhale_max : st_sampling_saved_buffer_index;
        st_sampling_algo_process_report(SAMPLE_TYPE_EXHALE_PREMAX, i, last);
        i = last;
        // Post-exhale-max samples
        last = st_sampling_saved_buffer_index_algo_exhale_min < st_sampling_saved_buffer_index ? st_sampling_saved_buffer_index_algo_exhale_min : st_sampling_saved_buffer_index;
        st_sampling_algo_process_report(SAMPLE_TYPE_EXHALE_POSTMAX, i, last);
        i = last;
        // Post-exhale-min samples
        last = st_sampling_saved_buffer_index;
        st_sampling_algo_process_report(SAMPLE_TYPE_EXHALE_POSTMIN, i, last);
        // End sample
        st_sampling_algo_process_report(SAMPLE_TYPE_END, last - 1, last);
        // Switch off sampling
        st_sampling_state = SAMPLING_STATE_NONE;
    }
}

static void st_sampling_process(void)
{
    if (!st_calibration_done)
    {
        st_calibration_done = 1;
        st_sampling_calibrate_process(st_sampling_frequency_for_calibration, st_sampling_frequency_for_calibration);
        return;
    }
    if (st_sampling_state == SAMPLING_STATE_FREE)
        st_sampling_free_process();
    else
        if (st_sampling_state == SAMPLING_STATE_ALGO)
            st_sampling_algo_process();
}

static void st_sampling_free_start(unsigned short frequency)
{
    if (!frequency)
        return;
    if (st_sampling_state == SAMPLING_STATE_FREE)
        if (frequency == st_sampling_frequency)
            return;
    st_sampling_frequency = frequency;
    if (st_sampling_frequency > SAMPLING_FREQUENCY_MAX)
        st_sampling_frequency = SAMPLING_FREQUENCY_MAX;
    st_sampling_period_ticks = (32768 + (st_sampling_frequency >> 1)) / st_sampling_frequency;
    st_sampling_state = SAMPLING_STATE_FREE;
    st_sampling_chunk_counter = 0;
    st_sampling_saved_buffer_index = 0;
    periodical_timer_set_frequency(1);
}

static void st_sampling_algo_start(
    int without_inhale,
    unsigned short duration,
    short threshold_inhale_start, short threshold_exhale_start,
    unsigned short threshold_trend_diff, unsigned short threshold_exhale_min,
    unsigned short threshold_silence_spread
    )
{
    if (!duration)
        return;
    st_sampling_without_inhale = without_inhale;
    st_sampling_frequency = SAMPLING_FREQUENCY_ALGO;
    st_sampling_duration = duration;
    st_sampling_threshold_inhale_start = threshold_inhale_start;
    st_sampling_threshold_exhale_start = threshold_exhale_start;
    st_sampling_threshold_trend_diff = threshold_trend_diff;
    st_sampling_threshold_exhale_min = threshold_exhale_min;
    st_sampling_threshold_silence_spread = threshold_silence_spread;
    if (st_sampling_duration > SAMPLING_DURATION_ALGO_MAX)
        st_sampling_duration = SAMPLING_DURATION_ALGO_MAX;
    st_sampling_algo_cycles_count = st_sampling_duration;
    st_sampling_algo_cycles_index = 0;
    periodical_timer_set_frequency(1);
    st_sampling_period_ticks = (32768 + (st_sampling_frequency >> 1)) / st_sampling_frequency;
    st_sampling_state = SAMPLING_STATE_ALGO;
    st_sampling_chunk_counter = 0;
    st_sampling_saved_buffer_index = 0;
    st_sampling_saved_buffer_index_algo_exhale_start = SAMPLING_SAVED_BUFFER_LEN;
    st_sampling_saved_buffer_index_algo_exhale_max = SAMPLING_SAVED_BUFFER_LEN;
    st_sampling_saved_buffer_index_algo_exhale_min = SAMPLING_SAVED_BUFFER_LEN;
    st_sampling_calibrate_process(st_sampling_frequency, SAMPLING_CALIBRATION_QUICK_COUNT);
    st_calibration_done = 1;
    st_set_averaged_period_in_ticks(st_sampling_period_ticks);
    st_decimation_init();
    st_sampling_saved_buffer_value_algo_start = st_get_decimated_measurement(0);
    st_sampling_saved_buffer_value_algo_exhale_min = 32767;
    st_sampling_saved_buffer_value_algo_exhale_max = -32768;
}

static void st_sampling_stop(void)
{
    periodical_timer_set_frequency(st_timer_frequency);
    st_sampling_state = SAMPLING_STATE_NONE;
    st_sampling_chunk_counter = 0;
    st_sampling_saved_buffer_index = 0;
}

#define OUTPUTS_COUNT 9
static unsigned char st_outputs[OUTPUTS_COUNT] =
    { LED_R1_PIN, LED_G1_PIN, LED_Y1_PIN, LED_R2_PIN, LED_G2_PIN, LED_Y2_PIN, DISABLE_CHG_PIN, SOLENOID_PIN, SUP_CLEAR_PIN };

static void st_version_report(void)
{
    int16_t version[2] = { CTRL_FIRMWARE_VERSION_MAJOR, CTRL_FIRMWARE_VERSION_MINOR };
    ser_command_send(OPCODE_VERSION, version, sizeof(version));
}

static void st_ser_input_process(void)
{
    unsigned short length;
    unsigned char payload[256], opcode;
    while (ser_command_receive(&opcode, payload, &length))
    {
        if (length && (length <= 256))
        {
            switch (opcode)
            {
                case OPCODE_SAMPLING:
                    if (length >= 2)
                    {
                        unsigned short frequency = *((unsigned short *)(&payload));
                        if (frequency)
                            st_sampling_free_start(frequency);
                        else
                            st_sampling_stop();
                    }
                    break;
                case OPCODE_OUTPUT_CONTROL:
                    {
                        int i;
                        i = 0;
                        while ((i < length) && (i < OUTPUTS_COUNT))
                        {
                            switch (payload[i])
                            {
                                case 0:
                                    nrf_gpio_pin_clear(st_outputs[i]);
                                    break;
                                case 1:
                                    nrf_gpio_pin_set(st_outputs[i]);
                                    break;
                                default:
                                    break;
                            }
                            ++i;
                        }
                    }
                    break;
                case OPCODE_DECIMATION_WINDOW_SIZE_UPDATE:
                    if (length >= 2)
                        st_set_decimation_window_size(*((unsigned short *)(&payload)));
                    ser_command_send(OPCODE_DECIMATION_WINDOW_SIZE_UPDATE, &st_decimation_window_size, sizeof(st_decimation_window_size));
                    break;
                case OPCODE_CALIBRATED_ZERO_GET:
                    ser_command_send(OPCODE_CALIBRATED_ZERO_GET, &st_sampling_calibrated_zero, sizeof(st_sampling_calibrated_zero));
                    break;
                case OPCODE_SAMPLING_ALGO_START:
                case OPCODE_SAMPLING_ALGO_START_WITHOUT_INHALE:
                    if (length >= 12)
                    {
                        int without_inhale = opcode == OPCODE_SAMPLING_ALGO_START_WITHOUT_INHALE ? 1 : 0;
                        unsigned short duration, threshold_trend_diff, threshold_exhale_min, threshold_silence_spread;
                        short threshold_inhale_start, threshold_exhale_start;
                        unsigned short *params = (unsigned short *)(&payload);
                        duration = params[0];
                        threshold_inhale_start = (short)params[1];
                        threshold_exhale_start = (short)params[2];
                        threshold_trend_diff = params[3];
                        threshold_exhale_min = params[4];
                        threshold_silence_spread = params[5];
                        st_sampling_algo_start(without_inhale, duration, threshold_inhale_start, threshold_exhale_start, threshold_trend_diff, threshold_exhale_min, threshold_silence_spread);
                    }
                    else
                        if (length >= 8) // Backward compatibility
                        {
                            int without_inhale = opcode == OPCODE_SAMPLING_ALGO_START_WITHOUT_INHALE ? 1 : 0;
                            unsigned short duration, threshold_trend_diff, threshold_exhale_min;
                            short threshold_inhale_start;
                            unsigned short *params = (unsigned short *)(&payload);
                            duration = params[0];
                            threshold_inhale_start = (short)params[1];
                            threshold_trend_diff = params[2];
                            threshold_exhale_min = params[3];
                            st_sampling_algo_start(without_inhale, duration, threshold_inhale_start, threshold_trend_diff, threshold_trend_diff, threshold_exhale_min, threshold_trend_diff);
                        }
                    break;
                case OPCODE_SAMPLING_ALGO_INDICES_GET:
                    {
                        unsigned short indices[SAMPLE_TYPE_END + 1];
                        indices[SAMPLE_TYPE_FREE] = 0;
                        indices[SAMPLE_TYPE_PRERECORD] = 0;
                        indices[SAMPLE_TYPE_INHALE] = 0;
                        indices[SAMPLE_TYPE_EXHALE_PREMAX] = st_sampling_saved_buffer_index_algo_exhale_start;
                        indices[SAMPLE_TYPE_EXHALE_POSTMAX] = st_sampling_saved_buffer_index_algo_exhale_max;
                        indices[SAMPLE_TYPE_EXHALE_POSTMIN] = st_sampling_saved_buffer_index_algo_exhale_min;
                        indices[SAMPLE_TYPE_END] = st_sampling_saved_buffer_index;
                        ser_command_send(OPCODE_SAMPLING_ALGO_INDICES_GET, indices, sizeof(indices));
                    }
                    break;
                case OPCODE_VERSION:
                    st_version_report();
                    break;
                case OPCODE_CONFIG_GET:
                    ser_command_send(OPCODE_CONFIG_GET, &st_sts_configuration, sizeof(sts_configuration_t));
                    break;
                case OPCODE_CONFIG_SET:
                    {
                        unsigned char ret = 0;
                        if (length >= sizeof(sts_configuration_t))
                        {
                            if (!memcmp(&st_sts_configuration, &payload, sizeof(sts_configuration_t)))
                                ret = 1;
                            else
                            {
                                memcpy(&st_sts_configuration, &payload, sizeof(sts_configuration_t));
                                if (st_save_configuration())
                                    ret = 1;
                                else
                                    st_load_configuration();
                            }
                        }
                        ser_command_send(OPCODE_CONFIG_SET, &ret, 1);
                    }
                    break;
                case OPCODE_BATTERY_LEVEL:
                    ser_command_send(OPCODE_BATTERY_LEVEL, &st_battery_level, sizeof(st_battery_level));
                    break;
                case OPCODE_SWITCH_TO_DFU:
                    if (payload[0] == 1)
                        cap_reset(1);
                    break;
                default:
                    break;
            }
        }
    }
}

//// Init

static void st_init_data(void)
{
    st_load_configuration();
    //
    //d_printf("Configuration.magic = %04X\r\n", st_sts_configuration.magic);
    //
}

//// Test

#define LEDS_TEST_COUNT 6
static uint8_t leds_test_array[LEDS_TEST_COUNT] = { LED_R1_PIN, LED_G1_PIN, LED_Y1_PIN, LED_R2_PIN, LED_G2_PIN, LED_Y2_PIN };
static uint8_t leds_test_index = LEDS_TEST_COUNT;

static void test(void)
{
#if 0
    d_printf("ADS8513: %d\r\n", ads8513_read());
    ++leds_test_index;
    if (leds_test_index >= LEDS_TEST_COUNT)
        leds_test_index = 0;
    nrf_gpio_pin_toggle(leds_test_array[leds_test_index]);
#else
    static int test_done = 0;
    uint64_t start_time, now;
    int count = 2000, i;
    if (test_done)
        return;
    test_done = 1;
    start_time = gl_rtc_ticks();
    for (i = 0; i < count; i++)
        ads8513_read();
    now = gl_rtc_ticks();
    now -= start_time;
    now = GL_RTC_TICKS_TO_MICROSECONDS(now);
    d_printf("ADS8513: sample rate is %d samples/sec\r\n", (int)(1000000 * count / now));
#endif
}

//// Loop

#define DISALLOW_BLE_CONNECTION_WHEN_USB_IS_CONNECTED

static int st_is_connected = 0;
static int st_loop_is_even = 0;

void ctrl_process_watchdog(void)
{
    periodical_timer_watchdog_feed();
}

void ctrl_process_irqs(void)
{
    // For drivers that requre a periodical state update
    gl_usb_process();
    st_ser_input_process();
}

void ctrl_process_request(unsigned char *data, unsigned short len)
{
    //char data_out[256];
    //int len_out = 0;
    //if (!cap_send((unsigned char *)data_out, (unsigned short)len_out, 0))
    //    return;
    //if (!cap_send(data, len, 0)) // Echo for testing
    //    return;
    ++st_report_counter;
    //
    st_ble_buf_store(data, len);
}

void ctrl_process_state(void)
{
    int timer_is_sleeping, usb_is_connected;
    if (!periodical_timer_counter_was_changed())
        return;
    st_chip_get_battery_level();
    st_chip_get_temperature();
    st_charger_monitor();
    ////
    //test();
    ////
    timer_is_sleeping = periodical_timer_is_sleeping();
    usb_is_connected = gl_usb_is_connected();
    if (usb_is_connected && timer_is_sleeping)
        periodical_timer_wakeup();
#ifdef DISALLOW_BLE_CONNECTION_WHEN_USB_IS_CONNECTED
    if (st_is_connected)
        if (usb_is_connected)
        {
            cap_disconnect(0);
            st_is_connected = 0;
        }
#endif
    st_sampling_process();
    if ((!st_is_connected) && (st_sampling_state == SAMPLING_STATE_NONE) && (!timer_is_sleeping) && (!usb_is_connected))
        periodical_timer_sleep();
    {
        int r = 0, y = 0, g = 0;
        if (st_is_connected)
            g = 1;
        else
            if (cap_is_adv_active())
                y = 1;
            else
                r = 1;
        st_led_ryg_set(1, r, y, g);
    }
    {
        int r = 0, y = 0, g = 0;
        if (st_is_charging())
            r = st_loop_is_even ? 0 : 1;
        else
            if (st_battery_decivolts < ST_BATTERY_WARNING_LEVEL_DECIVOLTS)
                r = 1;
            else
                g = 1;
        st_led_ryg_set(2, r, y, g);
        st_loop_is_even ^= 1;
    }
}

void ctrl_process_events(void)
{
}

void ctrl_on_connect(void)
{
    st_is_connected = 1;
#ifdef DISALLOW_BLE_CONNECTION_WHEN_USB_IS_CONNECTED
    if (gl_usb_is_connected())
    {
        cap_disconnect(0);
        st_is_connected = 0;
    }
#endif
    periodical_timer_wakeup();
}

void ctrl_on_disconnect(void)
{
    st_is_connected = 0;
    //periodical_timer_sleep();
}

void ctrl_on_adv_started(void)
{
}

void ctrl_on_adv_stopped(void)
{
}
