#ifndef GL_ROM_H__
#define GL_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


int gl_rom_is_available(void);

// NOTE: addr starts from 0, there is a four-bytes boundary, have to erase before rewrite

int gl_rom_read(unsigned int addr, void *data, unsigned int len);
int gl_rom_write(unsigned int addr, const void *data, unsigned int len);
int gl_rom_erase(void);


#ifdef __cplusplus
}
#endif

#endif // GL_ROM_H__
