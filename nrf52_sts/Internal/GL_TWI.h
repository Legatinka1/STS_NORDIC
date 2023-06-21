#ifndef GL_TWI_H__
#define GL_TWI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* TWI 0 */

void gl_twi_init(uint8_t sda_pin, uint8_t scl_pin);

uint8_t gl_twi_read_reg_byte(uint8_t address, uint8_t reg);
void gl_twi_write_reg_byte(uint8_t address, uint8_t reg, uint8_t value);

uint32_t gl_twi_read_reg(uint8_t address, uint8_t reg, uint8_t *buf, uint32_t len);
uint32_t gl_twi_write_reg(uint8_t address, uint8_t reg, const uint8_t *buf, uint32_t len);

int gl_twi_is_present(uint8_t address);

uint32_t gl_twi_read(uint8_t address, uint8_t *buf, uint32_t len);
uint32_t gl_twi_write(uint8_t address, const uint8_t *buf, uint32_t len);
uint32_t gl_twi_write_no_stop(uint8_t address, const uint8_t *buf, uint32_t len);

/* TWI 1 */

void gl_twi1_init(uint8_t sda_pin, uint8_t scl_pin);

uint8_t gl_twi1_read_reg_byte(uint8_t address, uint8_t reg);
void gl_twi1_write_reg_byte(uint8_t address, uint8_t reg, uint8_t value);

uint32_t gl_twi1_read_reg(uint8_t address, uint8_t reg, uint8_t *buf, uint32_t len);
uint32_t gl_twi1_write_reg(uint8_t address, uint8_t reg, const uint8_t *buf, uint32_t len);

int gl_twi1_is_present(uint8_t address);

uint32_t gl_twi1_read(uint8_t address, uint8_t *buf, uint32_t len);
uint32_t gl_twi1_write(uint8_t address, const uint8_t *buf, uint32_t len);
uint32_t gl_twi1_write_no_stop(uint8_t address, const uint8_t *buf, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif // GL_TWI_H__
