#ifndef _CAP_H_
#define _CAP_H_


#ifdef __cplusplus
extern "C" {
#endif


// Exposed BLE functions - implemented in main.c and may be called from anywhere

unsigned long long cap_get_own_mac_address(void);
unsigned long long cap_get_remote_mac_address(void);

int cap_start_adv(void);
int cap_is_adv_active(void);
void cap_stop_adv(void);
signed char cap_get_adv_tx_power(void);
signed char cap_validate_adv_tx_power(signed char tx_power);
signed char cap_set_adv_tx_power(signed char tx_power);

void cap_preinit_peer_manager(void);
int cap_is_connection_encrypted(void);

int cap_is_connected(void);
void cap_disconnect(int adv_on_disconnect_disabled);
int cap_send(const unsigned char *data, unsigned short len, int optionally);
int cap_get_rrsi(void);

void cap_hibernate(void);
void cap_reset(int go_to_bootloader);


// BLE configuration defaults - not a subject to change, they are intended to be updated dynamically in ctrl_*.c

#define CAP_DEFAULT_DEVICE_NAME "Cap_Name"
#define CAP_DEFAULT_ADV_INTERVAL 400 // 400 is 250 ms (unit is 0.625 ms)
#define CAP_DEFAULT_ADV_DURATION 1000 // 1000 is 10 s (unit is 100 ms, maximum value is 18000 that is 180 s)
#define CAP_DEFAULT_ADV_DATA_SERVICE_UUID 0x180F // Not present is 0, Device Information is 0x180A, Battery is 0x180F
#define CAP_DEFAULT_ADV_SCAN_RESPONSE_COMPANY_IDENTIFIER 0 // Not present is 0, Nordic is 0x0059, Apple is 0x004C - manufacturer specific data in scan response packet (beacon)
#define CAP_DEFAULT_ADV_SCAN_RESPONSE_SERVICE_UUID_EXPOSED 0 // 0 means false, 1 means true; Service 128-bit identifier in scan response packet; can't coexist with Company Identifier
#define CAP_DEFAULT_ADV_INITIALLY_ACTIVE 1 // 0 means false, 1 means true
#define CAP_DEFAULT_ADV_RESTART_ON_STOP 1 // 0 means false, 1 means true
#define CAP_DEFAULT_ADV_HIBERNATE_ON_STOP 0 // 0 means false, 1 means true; not relevant if Restart on Stop is true
#define CAP_DEFAULT_CONN_INTERVAL_MIN 8 // 8 is 10 ms (unit is 1.25 ms)
#define CAP_DEFAULT_CONN_INTERVAL_MAX 60 // 60 is 75 ms (unit is 1.25 ms)
#define CAP_DEFAULT_RSSI_MONITORING_ALLOWED 0 // 0 means false, 1 means true


// Hidden BLE functions - implemented in cap.c and should be called from main.c only

void cap_define_ble_behavior(
    const unsigned char mac_address[6],
    char device_name[32],
    unsigned long *adv_interval, // Default: 400 is 250 ms (unit is 0.625 ms)
    unsigned long *adv_duration, // Default: 1000 is 10 s (unit is 100 ms, maximum value is 18000 that is 180 s)
    unsigned short *adv_data_service_uuid, // Not present is 0, Default: 0x180F; Device Information is 0x180A, Battery is 0x180F
    unsigned short *adv_scan_response_company_identifier, // Default: 0 for none; Nordic is 0x0059, Apple is 0x004C - manufacturer specific data in scan response packet (beacon)
    int *adv_scan_response_service_uuid_exposed, // Default: 0; 0 means false, 1 means true; Service 128-bit identifier in scan response packet; can't coexist with Company Identifier
    int *adv_initially_active, // Default: 1; 0 means false, 1 means true
    int *adv_restart_on_stop, // Default: 1; 0 means false, 1 means true
    int *adv_hibernate_on_stop, // Default: 0; 0 means false, 1 means true; not relevant if Restart on Stop is true
    unsigned short *conn_interval_min, // Default: 8 is 10 ms (unit is 1.25 ms)
    unsigned short *conn_interval_max, // Default: 60 is 75 ms (unit is 1.25 ms)
    int *rssi_monitoring_allowed // Default: 0; 0 means false, 1 means true
    );

void cap_allow_custom_adv(
    const unsigned char *adv_name, unsigned short adv_name_len,
    const unsigned char *adv_data, unsigned short adv_data_len,
    const unsigned char *adv_man, unsigned short adv_man_len
    );

int cap_setup(void);
int cap_loop(int is_connected);

void cap_on_adv_started(void);
void cap_on_adv_stopped(void);

void cap_on_connect(void);
int cap_on_disconnect(void);
int cap_on_receive(const unsigned char *data, unsigned short len);


#ifdef __cplusplus
}
#endif

#endif /* _CAP_H_ */
