/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

////
#define CAP_PRESENT
#ifdef CAP_PRESENT
#include "cap.h"
#endif
////

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
#ifdef CAP_PRESENT
static unsigned char cap_own_mac_addr_type = 0;
static unsigned long long cap_own_mac_address = 0;
static unsigned long long cap_remote_mac_address = 0;
unsigned long long cap_get_own_mac_address(void)
{
    if (!cap_own_mac_address)
    {
        ble_gap_addr_t p_addr;
        sd_ble_gap_addr_get(&p_addr);
        memcpy(&cap_own_mac_address, p_addr.addr, 6);
        cap_own_mac_addr_type = p_addr.addr_type;
    }
    return cap_own_mac_address;
}
unsigned long long cap_get_remote_mac_address(void)
{
    return cap_remote_mac_address;
}
__WEAK void cap_define_ble_behavior(
    const unsigned char mac_address[6],
    char device_name[32],
    unsigned long *adv_interval,
    unsigned long *adv_duration,
    unsigned short *adv_data_service_uuid,
    unsigned short  *adv_scan_response_company_identifier,
    int *adv_scan_response_service_uuid_exposed,
    int *adv_initially_active,
    int *adv_restart_on_stop,
    int *adv_hibernate_on_stop,
    unsigned short *conn_interval_min,
    unsigned short *conn_interval_max,
    int *rssi_monitoring_allowed
    ) { }
static char cap_config_device_name[32] = "";
static unsigned long cap_config_adv_interval = 400; // 400 is 250 ms (unit is 0.625 ms)
static unsigned long cap_config_adv_duration = 1000; // 1000 is 10 s (unit is 100 ms, maximum value is 18000 that is 180 s)
static unsigned short cap_config_adv_data_service_uuid = 0x180F; // Not present is 0, Device Information is 0x180A, Battery is 0x180F
static unsigned short cap_config_adv_scan_response_company_identifier = 0; // Not present is 0, Nordic is 0x0059, Apple is 0x004C - manufacturer specific data in scan response packet (beacon)
static int cap_config_adv_scan_response_service_uuid_exposed = 0; // 0 means false, 1 means true; Service 128-bit identifier in scan response packet; can't coexist with Company Identifier
static int cap_config_adv_initially_active = 1; // 0 means false, 1 means true
static int cap_config_adv_restart_on_stop = 1; // 0 means false, 1 means true
static int cap_config_adv_hibernate_on_stop = 0; // 0 means false, 1 means true; not relevant if Restart on Stop is true
static unsigned short cap_config_conn_interval_min = 8; // 8 is 10 ms (unit is 1.25 ms)
static unsigned short cap_config_conn_interval_max = 60; // 60 is 75 ms (unit is 1.25 ms)
static int cap_config_rssi_monitoring_allowed = 0; // 0 means false, 1 means true
static void cap_config_ble(void)
{
    unsigned char mac_address[6];
    cap_get_own_mac_address();
    memcpy(mac_address, &cap_own_mac_address, 6);
    NRF_LOG_INFO("MAC address: %02X:%02X:%02X:%02X:%02X:%02X",
        mac_address[5], mac_address[4], mac_address[3], mac_address[2], mac_address[1], mac_address[0]);
#ifdef CAP_DEFAULT_DEVICE_NAME
    strncpy(cap_config_device_name, CAP_DEFAULT_DEVICE_NAME, sizeof(cap_config_device_name));
    cap_config_device_name[sizeof(cap_config_device_name) - 1] = 0;
#endif
#ifdef CAP_DEFAULT_ADV_INTERVAL
    cap_config_adv_interval = CAP_DEFAULT_ADV_INTERVAL;
#endif
#ifdef CAP_DEFAULT_ADV_DURATION
    cap_config_adv_duration = CAP_DEFAULT_ADV_DURATION;
#endif
#ifdef CAP_DEFAULT_ADV_DATA_SERVICE_UUID
    cap_config_adv_data_service_uuid = CAP_DEFAULT_ADV_DATA_SERVICE_UUID;
#endif
#ifdef CAP_DEFAULT_ADV_SCAN_RESPONSE_COMPANY_IDENTIFIER
    cap_config_adv_scan_response_company_identifier = CAP_DEFAULT_ADV_SCAN_RESPONSE_COMPANY_IDENTIFIER;
#endif
#ifdef CAP_DEFAULT_ADV_SCAN_RESPONSE_SERVICE_UUID_EXPOSED
    cap_config_adv_scan_response_service_uuid_exposed = CAP_DEFAULT_ADV_SCAN_RESPONSE_SERVICE_UUID_EXPOSED;
#endif
#ifdef CAP_DEFAULT_ADV_INITIALLY_ACTIVE
    cap_config_adv_initially_active = CAP_DEFAULT_ADV_INITIALLY_ACTIVE;
#endif
#ifdef CAP_DEFAULT_ADV_RESTART_ON_STOP
    cap_config_adv_restart_on_stop = CAP_DEFAULT_ADV_RESTART_ON_STOP;
#endif
#ifdef CAP_DEFAULT_ADV_HIBERNATE_ON_STOP
    cap_config_adv_hibernate_on_stop = CAP_DEFAULT_ADV_HIBERNATE_ON_STOP;
#endif
#ifdef CAP_DEFAULT_CONN_INTERVAL_MIN
    cap_config_conn_interval_min = CAP_DEFAULT_CONN_INTERVAL_MIN;
#endif
#ifdef CAP_DEFAULT_CONN_INTERVAL_MAX
    cap_config_conn_interval_max = CAP_DEFAULT_CONN_INTERVAL_MAX;
#endif
#ifdef CAP_DEFAULT_RSSI_MONITORING_ALLOWED
    cap_config_rssi_monitoring_allowed = CAP_DEFAULT_RSSI_MONITORING_ALLOWED;
#endif
    if (!(*cap_config_device_name))
        sprintf(cap_config_device_name, "%c_%02X%02X%02X",
            cap_own_mac_addr_type == BLE_GAP_ADDR_TYPE_PUBLIC ? 'P' : 'R',
            mac_address[2], mac_address[1], mac_address[0]);
    cap_define_ble_behavior(
        mac_address,
        cap_config_device_name,
        &cap_config_adv_interval,
        &cap_config_adv_duration,
        &cap_config_adv_data_service_uuid,
        &cap_config_adv_scan_response_company_identifier,
        &cap_config_adv_scan_response_service_uuid_exposed,
        &cap_config_adv_initially_active,
        &cap_config_adv_restart_on_stop,
        &cap_config_adv_hibernate_on_stop,
        &cap_config_conn_interval_min,
        &cap_config_conn_interval_max,
        &cap_config_rssi_monitoring_allowed
        );
    NRF_LOG_INFO("Device name: \"%s\"", cap_config_device_name);
}
#endif
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
#ifdef CAP_PRESENT
    cap_config_ble();
#endif

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

#ifdef CAP_PRESENT
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) cap_config_device_name,
                                          strlen(cap_config_device_name));
#else
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
#endif

    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

#ifdef CAP_PRESENT
    gap_conn_params.min_conn_interval = cap_config_conn_interval_min;
    gap_conn_params.max_conn_interval = cap_config_conn_interval_max;
#else
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
#endif
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
#ifdef CAP_PRESENT
        cap_on_receive(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
#else
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
#endif
    }

}
/**@snippet [Handling the data received over BLE] */
#ifdef CAP_PRESENT
int cap_send(const unsigned char *data, unsigned short len, int optionally)
{
    uint32_t err_code;
    uint16_t length;
    do
    {
        length = (uint16_t)len;
        err_code = ble_nus_data_send(&m_nus, (uint8_t *)data, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            //APP_ERROR_CHECK(err_code);
        }
    } while ((err_code == NRF_ERROR_RESOURCES) && (!optionally));
    return err_code == NRF_SUCCESS ? 1 : 0;
}
__WEAK int cap_on_receive(const unsigned char *data, unsigned short len)
{
    return 0;
}
#endif


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
#ifndef CAP_PRESENT
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
#endif
        //APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
#ifdef CAP_PRESENT
    uint32_t err_code;
#else
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
#ifdef CAP_PRESENT
__WEAK void cap_on_adv_started(void)
{
}
__WEAK void cap_on_adv_stopped(void)
{
}
static int cap_adv_active = 0;
int cap_start_adv(void)
{
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        return 0;
    if (cap_adv_active)
        return 1;
    m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = false;
    cap_adv_active = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST) == NRF_SUCCESS ? 1 : 0;
    return cap_adv_active;
}
int cap_is_adv_active(void)
{
    return cap_adv_active;
}
void cap_stop_adv(void)
{
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        return;
    sd_ble_gap_adv_stop(m_advertising.adv_handle);
    cap_adv_active = 0;
}
void cap_hibernate(void)
{
    sleep_mode_enter();
}
void cap_reset(int go_to_bootloader)
{
    if (go_to_bootloader)
    {
        sd_power_gpregret_clr(0, 0xFF);
        sd_power_gpregret_set(0, 0xB1/*BOOTLOADER_DFU_START*/);
        //m_dfu.evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER);
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
    }
    else
    {
        app_error_handler_bare(3);
    }
}
#endif
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
#ifdef CAP_PRESENT
            cap_adv_active = 1;
            cap_on_adv_started();
#else
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
#endif
            break;
        case BLE_ADV_EVT_IDLE:
#ifdef CAP_PRESENT
            cap_adv_active = 0;
            cap_on_adv_stopped();
            if (cap_config_adv_restart_on_stop)
                cap_start_adv();
            else
                if (cap_config_adv_hibernate_on_stop)
                    sleep_mode_enter();
#else
            sleep_mode_enter();
#endif
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
#ifdef CAP_PRESENT
static int8_t cap_rssi_value = 0;
static uint8_t cap_rssi_ch_index = 0;
int cap_get_rrsi(void)
{
    return cap_rssi_value;
}
static signed char cap_adv_tx_power = 0;
signed char cap_get_adv_tx_power(void)
{
    return cap_adv_tx_power;
}
signed char cap_validate_adv_tx_power(signed char tx_power)
{
    signed char p;
    if (tx_power <= -40) p = -40; else
    if (tx_power <= -20) p = -20; else
    if (tx_power <= -16) p = -16; else
    if (tx_power <= -12) p = -12; else
    if (tx_power <= -8) p = -8; else
    if (tx_power <= -4) p = -4; else
    if (tx_power <= 0) p = 0; else
    if (tx_power <= 3) p = 3; else
    p = 4;
    return p;
}
signed char cap_set_adv_tx_power(signed char tx_power)
{
    signed char p = cap_validate_adv_tx_power(tx_power);
    if (p != cap_adv_tx_power)
    {
        if (sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, p) == NRF_SUCCESS)
            cap_adv_tx_power = p;
    }
    return cap_adv_tx_power;
}
static int cap_encrypted_connection_is_allowed = 0;
void cap_preinit_peer_manager(void)
{
    cap_encrypted_connection_is_allowed = 1;
}
int cap_is_connection_encrypted(void)
{
    return ble_conn_state_encrypted(m_conn_handle) ? 1 : 0;
}
    //  //// Security
    //
    //      // Include directories components/ble/peer_manager and components/libraries/fds
    //      // Add to "nRF_Libraries" file components/libraries/fds/fds.c
    //      // Add to "nRF_BLE" files components/ble/peer_manager/peer_data_storage.c, peer_database.c, peer_id.c, peer_manager.c, peer_manager_handler.c, pm_buffer.c, security_dispatcher.c and security_manager.c
    //      // In sdk_config.h set PEER_MANAGER_ENABLED 1, PM_CENTRAL_ENABLED 0 and FDS_ENABLED 1
    //
    //  #include "peer_manager.h"
    //  #include "peer_manager_handler.h"
    //
    //  static void pm_evt_handler(pm_evt_t const * p_evt)
    //  {
    //      pm_handler_on_pm_evt(p_evt);
    //      pm_handler_flash_clean(p_evt);
    //  }
    //
    //  static void peer_manager_init()
    //  {
    //      ble_gap_sec_params_t sec_param;
    //      // Disable main handling of BLE events BLE_GAP_EVT_SEC_PARAMS_REQUEST and BLE_GATTS_EVT_SYS_ATTR_MISSING
    //      cap_preinit_peer_manager();
    //      pm_init();
    //      memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    //      // Pairing, no bonding
    //      sec_param.io_caps = BLE_GAP_IO_CAPS_NONE;
    //      sec_param.min_key_size = 7;
    //      sec_param.max_key_size = 16;
    //      pm_sec_params_set(&sec_param);
    //      pm_register(pm_evt_handler);
    //  }
__WEAK void cap_on_connect(void)
{
}
__WEAK int cap_on_disconnect(void)
{
    return 0;
}
int cap_is_connected(void)
{
    return m_conn_handle == BLE_CONN_HANDLE_INVALID ? 0 : 1;
}
void cap_disconnect(int adv_on_disconnect_disabled)
{
    m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = adv_on_disconnect_disabled ? true : false;
    sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}
#endif
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
#ifndef CAP_PRESENT
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
#endif
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
#ifdef CAP_PRESENT
            m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = false;
            cap_adv_active = 0;
            if (cap_config_rssi_monitoring_allowed)
                sd_ble_gap_rssi_start(m_conn_handle, 1, 0);  
            memcpy(&cap_remote_mac_address, p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr, 6);
            cap_on_connect();
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
#ifdef CAP_PRESENT
            cap_remote_mac_address = 0;
            cap_on_disconnect();
#endif
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
#ifdef CAP_PRESENT
            if (!cap_encrypted_connection_is_allowed)
            {
#endif
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
#ifdef CAP_PRESENT
            }
#endif
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
#ifdef CAP_PRESENT
            if (!cap_encrypted_connection_is_allowed)
            {
#endif
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
#ifdef CAP_PRESENT
            }
#endif
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
#ifdef CAP_PRESENT
        case BLE_GAP_EVT_RSSI_CHANGED:
            if (cap_config_rssi_monitoring_allowed)
            {
                err_code = sd_ble_gap_rssi_get(p_ble_evt->evt.gatts_evt.conn_handle, &cap_rssi_value, &cap_rssi_ch_index);
                APP_ERROR_CHECK(err_code);
            }
            break;
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
#ifndef CAP_PRESENT
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}
#endif


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
#ifndef CAP_PRESENT
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
#endif
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
#ifndef CAP_PRESENT
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    //APP_ERROR_CHECK(err_code);
#endif
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
#ifdef CAP_PRESENT
__WEAK void cap_allow_custom_adv(
    const unsigned char *adv_name, unsigned short adv_name_len,
    const unsigned char *adv_data, unsigned short adv_data_len,
    const unsigned char *adv_man, unsigned short adv_man_len
    )
{
}
#endif
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
#ifdef CAP_PRESENT
    uint8_t zeros[32];
    uint16_t name_len = strlen(cap_config_device_name);
    uint16_t data_len = cap_config_adv_data_service_uuid ? (name_len >= 22 ? 0 : 22 - name_len) : 0;
    uint16_t man_len = 0;
    uint16_t data_offset = data_len ? 7 : 3;
    uint16_t name_offset = data_len ? 9 + data_len : 5;
    uint16_t man_offset = 0;
    ble_advdata_service_data_t service_data;
    ble_advdata_manuf_data_t manuf_specific_data;
    memset(&zeros, 0, sizeof(zeros));
#endif

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
#ifdef CAP_PRESENT
    if (cap_config_adv_data_service_uuid && data_len)
    {
        service_data.service_uuid = cap_config_adv_data_service_uuid;
        service_data.data.p_data = zeros;
        service_data.data.size = data_len;
        init.advdata.p_service_data_array = &service_data;
        init.advdata.service_data_count = 1;
    }
#endif

#ifdef CAP_PRESENT
    if (cap_config_adv_scan_response_company_identifier)
    {
        man_len = 23;
        man_offset = 4;
        manuf_specific_data.company_identifier = cap_config_adv_scan_response_company_identifier;
        manuf_specific_data.data.p_data = zeros;
        manuf_specific_data.data.size = man_len;
        init.srdata.p_manuf_specific_data = &manuf_specific_data;
    }
    else
    {
        if (cap_config_adv_scan_response_service_uuid_exposed)
        {
            man_len = 16;
            man_offset = 2;
            init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
            init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
        }
    }
#else
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
#endif

    init.config.ble_adv_fast_enabled  = true;
#ifdef CAP_PRESENT
    init.config.ble_adv_fast_interval = cap_config_adv_interval;
    init.config.ble_adv_fast_timeout  = cap_config_adv_duration;
#else
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
#endif
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    //APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
#ifdef CAP_PRESENT
    cap_allow_custom_adv(
        m_advertising.adv_data.adv_data.p_data + name_offset, name_len,
        m_advertising.adv_data.adv_data.p_data + data_offset, data_len,
        man_len ? m_advertising.adv_data.scan_rsp_data.p_data + man_offset : 0, man_len
        );
    ////By ble_advertising.c/ble_advertising_advdata_update():
    //m_advertising.p_adv_data = &m_advertising.adv_data;
    //sd_ble_gap_adv_set_configure(&m_advertising.adv_handle, m_advertising.p_adv_data, NULL);
#endif
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
#ifndef CAP_PRESENT
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
#endif
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
#ifdef CAP_PRESENT
    cap_start_adv();
#else
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief Application main function.
 */
#ifdef CAP_PRESENT
#include "nrf_drv_power.h"
__WEAK int cap_setup(void)
{
    return 0;
}
__WEAK int cap_loop(int is_connected)
{
    return 0;
}
__WEAK void cap_on_interrupt(int id)
{
}
#endif
int main(void)
{
    bool erase_bonds;

    // Initialize.
    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
#ifdef CAP_PRESENT
#if NRF_MODULE_ENABLED(POWER)
    nrf_drv_power_init(0); // Need for USB
#endif
#endif

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
#ifndef CAP_PRESENT
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
#endif
#ifdef CAP_PRESENT
    cap_setup();
    if (cap_config_adv_initially_active)
#endif
    advertising_start();

    // Enter main loop.
    for (;;)
    {
#ifdef CAP_PRESENT
        cap_loop(m_conn_handle == BLE_CONN_HANDLE_INVALID ? 0 : 1);
#endif
        idle_state_handle();
    }
}


/**
 * @}
 */
