#ifndef _CTRL_H_
#define _CTRL_H_


#ifdef __cplusplus
extern "C" {
#endif


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
    ); // Look at cap.h for the default values

void ctrl_fill_adv_name(char *name, unsigned short len);
void ctrl_fill_adv_data(unsigned char *data, unsigned short len);
void ctrl_fill_adv_man(unsigned char *data, unsigned short len);

void ctrl_init(void);
void ctrl_begin(void);

void ctrl_process_watchdog(void);
void ctrl_process_irqs(void);
void ctrl_process_request(unsigned char *data, unsigned short len);
void ctrl_process_state(void);
void ctrl_process_events(void);

void ctrl_on_connect(void);
void ctrl_on_disconnect(void);

void ctrl_on_adv_started(void);
void ctrl_on_adv_stopped(void);


#ifdef __cplusplus
}
#endif

#endif /* _CTRL_H_ */

