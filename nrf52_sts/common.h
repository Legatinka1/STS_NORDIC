#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


/* Pins */

#include "pins.h"

/* Debug */

#define USE_DEBUG_OUTPUT_NONE 0
#define USE_DEBUG_OUTPUT_UART 1
#define USE_DEBUG_OUTPUT_TWI 2
#define USE_DEBUG_OUTPUT_SWO 3
#define USE_DEBUG_OUTPUT_USB 4
#define USE_DEBUG_OUTPUT_RTT 5
#define USE_DEBUG_OUTPUT_UART1 6

#define USE_DEBUG_OUTPUT_DEFAULT USE_DEBUG_OUTPUT_RTT

extern int use_debug_output/* = USE_DEBUG_OUTPUT_DEFAULT*/;

void d_prin(const void *buf, int len);
void d_print(const char *s);
void d_printf(const char *fmt, ...);

int d_getchar(void);

void d_flush(void);

/* Timer */

void periodical_timer_set_period(unsigned short period);
unsigned short periodical_timer_get_period(void);
void periodical_timer_set_frequency(unsigned short frequency);
unsigned short periodical_timer_get_frequency(void);
void periodical_timer_sleep(void);
int periodical_timer_is_sleeping(void);
void periodical_timer_wakeup(void);
unsigned short periodical_timer_get_counter(void);
int periodical_timer_counter_was_changed(void);
void periodical_timer_set_watchdog_timeout(unsigned long timeout);
void periodical_timer_watchdog_feed(void);

/* Interrupt */

int lotohi_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int));
int lotohi_interrupt_is_enabled(void);
int lotohi_interrupt_was_latched(void);
void lotohi_interrupt_disable(void);

int hitolo_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int));
int hitolo_interrupt_is_enabled(void);
int hitolo_interrupt_was_latched(void);
void hitolo_interrupt_disable(void);

int toggle_interrupt_enable(unsigned char pin, int pull, int id, void (*callback)(int));
int toggle_interrupt_is_enabled(void);
int toggle_interrupt_was_latched(void);
void toggle_interrupt_disable(void);

/* Wakeup */

int lotohi_wakeup_enable(unsigned char pin, int pull);

int hitolo_wakeup_enable(unsigned char pin, int pull);

int toggle_wakeup_enable(unsigned char pin, int pull);

/* Helper */

void swap_shorts(void *data, int count);
void swap_longs(void *data, int count);

unsigned char crcae(const void *data, unsigned int len);

/* Internals */

#include "Internal/GL_RTC.h"
#include "Internal/GL_TWI.h"
#include "Internal/GL_SPI.h"
#include "Internal/GL_SDC.h"
#include "Internal/GL_ADC.h"
#include "Internal/GL_ROM.h"
#include "Internal/GL_USB.h"
#include "Internal/GL_COM.h"
#include "Internal/GL_I2S.h"
#include "Internal/GL_PWM.h"


#ifdef __cplusplus
}
#endif

#endif /* _COMMON_H_ */
