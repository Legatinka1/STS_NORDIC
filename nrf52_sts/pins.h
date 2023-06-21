#ifndef _PINS_H_
#define _PINS_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Pins */

#include "boards.h"

#define GEN_TX_PIN           6
#define GEN_RX_PIN           8
#define GEN_BAUDRATE    115200

#define GEN_SDA_PIN         26
#define GEN_SCL_PIN         27

#include "nrf_saadc.h"

#define VIN_CPU               NRF_SAADC_INPUT_VDD

//#define AIN_VBAT_INPUT        NRF_SAADC_INPUT_AIN0 // Pin 2
//#define AIN_VBAT_R_UP         187 // v = ((v * (AIN_VBAT_R_UP + AIN_VBAT_R_DOWN)) + (AIN_VBAT_R_DOWN >> 1)) / AIN_VBAT_R_DOWN
//#define AIN_VBAT_R_DOWN       274 // v = ((v * (AIN_VBAT_R_UP + AIN_VBAT_R_DOWN)) + (AIN_VBAT_R_DOWN >> 1)) / AIN_VBAT_R_DOWN

#define ANALOG_PINS_COUNT 1
#define ANALOG_PINS { VIN_CPU }
#define ANALOG_PIN_VDD_INDEX 0


#ifdef __cplusplus
}
#endif

#endif /* _PINS_H_ */
