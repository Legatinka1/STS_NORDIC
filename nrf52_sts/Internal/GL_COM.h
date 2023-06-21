#ifndef GL_COM_H__
#define GL_COM_H__

#ifdef __cplusplus
extern "C" {
#endif


#define GL_COM_PIN_DISCONNECTED 0xFF

/* UART 0 */

int gl_com_init(unsigned char tx_pin, unsigned char rx_pin, unsigned long baudrate, int polling);
int gl_com_is_initialized(void);
int gl_com_uninit(void);

void gl_com_set_custom_receive_handler(void (*on_receive)(unsigned char));

unsigned long gl_com_read(void *buf, unsigned long len);
unsigned long gl_com_write(const void *buf, unsigned long len);
void gl_com_flush(void);

/* UART 1 */

int gl_com1_init(unsigned char tx_pin, unsigned char rx_pin, unsigned long baudrate, int polling);
int gl_com1_is_initialized(void);
int gl_com1_uninit(void);

void gl_com1_set_custom_receive_handler(void (*on_receive)(unsigned char));

unsigned long gl_com1_read(void *buf, unsigned long len);
unsigned long gl_com1_write(const void *buf, unsigned long len);
void gl_com1_flush(void);


#ifdef __cplusplus
}
#endif

#endif // GL_COM_H__
