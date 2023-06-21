#ifndef GL_USB_H__
#define GL_USB_H__

#ifdef __cplusplus
extern "C" {
#endif


void gl_usb_init(void);
void gl_usb_process(void);

int gl_usb_is_connected(void);
unsigned short gl_usb_write(const void *data, unsigned short len, int optionally);
unsigned short gl_usb_get_readable_count(void);
unsigned short gl_usb_read(void *data, unsigned short count, int peek);
unsigned short gl_usb_peek(unsigned short pos, unsigned char *data);
unsigned short gl_usb_get_free_space_for_read(void);
void gl_usb_flush(void);


#ifdef __cplusplus
}
#endif

#endif // GL_USB_H__
