#include "cap.h"

#include "compiler_abstraction.h"

//#include "tests.h"
__WEAK int test_in_setup(int param) { return 0; }
__WEAK int test_in_loop(int param) { return 0; }

//#include "ctrl.h"
__WEAK void ctrl_define_ble_behavior(
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
    ) { }
__WEAK void ctrl_fill_adv_name(char *name, unsigned short len) { }
__WEAK void ctrl_fill_adv_data(unsigned char *data, unsigned short len) { }
__WEAK void ctrl_fill_adv_man(unsigned char *data, unsigned short len) { }
__WEAK void ctrl_init(void) { }
__WEAK void ctrl_begin(void) { }
__WEAK void ctrl_process_irqs(void) { }
__WEAK void ctrl_process_request(unsigned char *data, unsigned short len) { }
__WEAK void ctrl_process_state(void) { }
__WEAK void ctrl_process_events(void) { }
__WEAK void ctrl_process_watchdog(void) { }
__WEAK void ctrl_on_connect(void) { }
__WEAK void ctrl_on_disconnect(void) { }
__WEAK void ctrl_on_adv_started(void) { }
__WEAK void ctrl_on_adv_stopped(void) { }

void cap_define_ble_behavior(
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
    ctrl_define_ble_behavior(
        mac_address,
        device_name,
        adv_interval,
        adv_duration,
        adv_data_service_uuid,
        adv_scan_response_company_identifier,
        adv_scan_response_service_uuid_exposed,
        adv_initially_active,
        adv_restart_on_stop,
        adv_hibernate_on_stop,
        conn_interval_min,
        conn_interval_max,
        rssi_monitoring_allowed
        );
}

static int st_cap_is_connected = 0;

static int st_adv_name_allowed = 0;
static unsigned char st_dummy_adv_name[8];
static unsigned char *st_adv_name = st_dummy_adv_name;
static unsigned short st_adv_name_len = sizeof(st_dummy_adv_name);
static int st_adv_data_allowed = 0;
static unsigned char st_dummy_adv_data[14];
static unsigned char *st_adv_data = st_dummy_adv_data;
static unsigned short st_adv_data_len = sizeof(st_dummy_adv_data);
static int st_adv_man_allowed = 0;
static unsigned char st_dummy_adv_man[23];
static unsigned char *st_adv_man = st_dummy_adv_man;
static unsigned short st_adv_man_len = sizeof(st_dummy_adv_man);

static void cap_update_custom_adv(void)
{
    if (st_adv_name_allowed)
        ctrl_fill_adv_name((char *)st_adv_name, st_adv_name_len);
    if (st_adv_data_allowed)
        ctrl_fill_adv_data(st_adv_data, st_adv_data_len);
    if (st_adv_man_allowed)
        ctrl_fill_adv_man((char *)st_adv_man, st_adv_man_len);
}

void cap_allow_custom_adv(
    const unsigned char *adv_name, unsigned short adv_name_len,
    const unsigned char *adv_data, unsigned short adv_data_len,
    const unsigned char *adv_man, unsigned short adv_man_len
    )
{
    st_adv_name_allowed = adv_name && adv_name_len;
    if (st_adv_name_allowed)
    {
        st_adv_name = (unsigned char *)adv_name;
        st_adv_name_len = adv_name_len;
    }
    st_adv_data_allowed = adv_data && adv_data_len;
    if (st_adv_data_allowed)
    {
        st_adv_data = (unsigned char *)adv_data;
        st_adv_data_len = adv_data_len;
    }
    st_adv_man_allowed = adv_man && adv_man_len;
    if (st_adv_man_allowed)
    {
        st_adv_man = (unsigned char *)adv_man;
        st_adv_man_len = adv_man_len;
    }
    cap_update_custom_adv();
}

static int st_cap_is_set = 0;

int cap_setup(void)
{
    if (st_cap_is_set)
        return 1;
    ctrl_init();
    ctrl_begin();
    test_in_setup(0);
    st_cap_is_set = 1;
    return st_cap_is_set;
}

void cap_on_connect(void)
{
}

int cap_on_disconnect(void)
{
    return 1;
}

#include <string.h>

#define CAP_MAX_MSGLEN 256

static unsigned char st_received_data_buf[2][CAP_MAX_MSGLEN];
static unsigned short st_received_data_buf_len[2] = { 0, 0 };
static int st_received_data_buf_num = 0;

int cap_on_receive(const unsigned char *data, unsigned short len)
{
   if ((!data) || (!len) || (len > CAP_MAX_MSGLEN))
        return 0;
    memcpy(st_received_data_buf[st_received_data_buf_num], data, len);
    st_received_data_buf_len[st_received_data_buf_num] = len;
    return 1;
}

static void st_clear_received(void)
{
    st_received_data_buf_num = 0;
    st_received_data_buf_len[0] = 0;
    st_received_data_buf_len[1] = 0;
}

static int st_process_received(void)
{
    int num = st_received_data_buf_num;
    unsigned char *data = st_received_data_buf[num];
    unsigned short len = st_received_data_buf_len[num];
    if (!len)
        return 0;
    st_received_data_buf_num ^= 1;
    st_received_data_buf_len[num] = 0;
    ctrl_process_request(data, len);
    return 1;
}

static int st_process_events(void)
{
    ctrl_process_events();
    return 0;
}

static void st_adv_process(void)
{
    if (cap_is_adv_active())
        cap_update_custom_adv();
}

static int st_process_state(int is_connected)
{
    st_adv_process();
    ctrl_process_state();
    return 0;
}

int cap_loop(int is_connected)
{
    int ret = 0;
    ctrl_process_watchdog();
    ctrl_process_irqs();
    if (is_connected)
    {
        if (!st_cap_is_connected)
        {
            st_cap_is_connected = 1;
            ctrl_on_connect();
        }
        ret = st_process_received();
        if (!ret)
        {
            ret = st_process_state(is_connected);
            ret |= st_process_events();
        }
    }
    else
    {
        if (st_cap_is_connected)
        {
            st_cap_is_connected = 0;
            ctrl_on_disconnect();
            st_clear_received();
            ret = st_process_state(is_connected);
        }
        else
        {
            ret = st_process_state(is_connected);
        }
    }
    test_in_loop(0);
    return ret;
}

void cap_on_adv_started(void)
{
    cap_update_custom_adv();
    ctrl_on_adv_started();
}

void cap_on_adv_stopped(void)
{
    ctrl_on_adv_stopped();
}
