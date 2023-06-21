#ifndef GL_I2S_H__
#define GL_I2S_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#define GL_I2S_PIN_DISCONNECTED 0xFF

void gl_i2s_init(uint8_t mck_pin, uint8_t sck_pin, uint8_t lrck_pin, uint8_t sdout_pin);

int gl_i2s_play_tone(unsigned long period_us, unsigned long duration_us);
int gl_i2s_play_buffer(const short *buf, unsigned long buf_samples_count, int is_mono, int do_loop);
int gl_i2s_is_playing(void);
void gl_i2s_stop_playing(void);


#ifdef __cplusplus
}
#endif

#endif // GL_I2S_H__
