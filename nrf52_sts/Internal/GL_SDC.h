#ifndef GL_SDC_H__
#define GL_SDC_H__

#ifdef __cplusplus
extern "C" {
#endif


void gl_sdc_set_spi_pins(unsigned char mosi_pin, unsigned char miso_pin, unsigned char sck_pin, unsigned char cs_pin);
void gl_sdc_set_wait_func(void (*wait_func)(void));

unsigned char gl_sdc_init(void);

unsigned char gl_sdc_status(void);

unsigned long gl_sdc_capacity_in_megabytes(void);

unsigned char gl_sdc_uninit(void);
unsigned char gl_sdc_reinit(void);


#ifdef __cplusplus
}
#endif

#endif // GL_SDC_H__
