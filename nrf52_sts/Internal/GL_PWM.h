#ifndef GL_PWM_H__
#define GL_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif


#define GL_PWM_PIN_DISCONNECTED 0xFF

void gl_pwm_set_pins(unsigned char drv, unsigned char pin0, unsigned char pin1, unsigned char pin2, unsigned char pin3);

int gl_pwm_play_tone(unsigned char drv, unsigned long period_us, unsigned long duration_us); // period_us: 1..16383 (0x0001..0x3FFF) - 1 MHz down to 61 Hz
int gl_pwm_play_value(unsigned char drv, unsigned short period_us, unsigned short duty_cycle_us); // period_us: 1..32768 (0x0000..0x8000) - up to 15 bits, duty_cycle_us: 0..period_us-1
int gl_pwm_is_playing(unsigned char drv);
void gl_pwm_stop_playing(unsigned char drv);


#ifdef __cplusplus
}
#endif

#endif // GL_PWM_H__
