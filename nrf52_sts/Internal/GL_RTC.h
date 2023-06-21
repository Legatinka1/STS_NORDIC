#ifndef GL_RTC_H__
#define GL_RTC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define GL_RTC_TICKS_PER_SECOND 32768

#define GL_RTC_TICKS_TO_SECONDS(ticks) ((uint64_t)((ticks) >> 15))
#define GL_RTC_TICKS_TO_MILLISECONDS100(ticks) ((uint64_t)(((ticks) * 5ULL) >> 14))
#define GL_RTC_TICKS_TO_MILLISECONDS10(ticks) ((uint64_t)(((ticks) * 25ULL) >> 13))
#define GL_RTC_TICKS_TO_MILLISECONDS(ticks) ((uint64_t)(((ticks) * 125ULL) >> 12))
#define GL_RTC_TICKS_TO_MICROSECONDS100(ticks) ((uint64_t)(((ticks) * 625ULL) >> 11))
#define GL_RTC_TICKS_TO_MICROSECONDS10(ticks) ((uint64_t)(((ticks) * 3125ULL) >> 10))
#define GL_RTC_TICKS_TO_MICROSECONDS(ticks) ((uint64_t)(((ticks) * 15625ULL) >> 9))

uint32_t gl_rtc_config(void);

uint64_t gl_rtc_ticks(void);
uint64_t gl_rtc_seconds(void);
uint64_t gl_rtc_milliseconds(void);
uint64_t gl_rtc_microseconds(void);

void gl_rtc_set_virtual_ticks(uint64_t virtual_ticks);
int64_t gl_rtc_get_virtual_ticks_offset(void);
void gl_rtc_set_virtual_ticks_of_ticks(uint64_t virtual_ticks, uint64_t ticks);
uint64_t gl_rtc_get_virtual_ticks_of_ticks(uint64_t ticks);

uint64_t gl_rtc_virtual_ticks(void);
uint64_t gl_rtc_virtual_seconds(void);
uint64_t gl_rtc_virtual_milliseconds(void);
uint64_t gl_rtc_virtual_microseconds(void);


#ifdef __cplusplus
}
#endif

#endif // GL_RTC_H__
