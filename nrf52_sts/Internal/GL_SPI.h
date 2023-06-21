#ifndef GL_SPI_H__
#define GL_SPI_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* SPI 1 */

void gl_spi_init(uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t mode);

void gl_spi_prepare_ss_pin(uint8_t ss_pin);

uint8_t gl_spi_read_reg_byte(uint8_t ss_pin, uint8_t reg);
void gl_spi_write_reg_byte(uint8_t ss_pin, uint8_t reg, uint8_t value);

uint32_t gl_spi_read_reg(uint8_t ss_pin, uint8_t reg, uint8_t *buf, uint32_t len);
uint32_t gl_spi_write_reg(uint8_t ss_pin, uint8_t reg, const uint8_t *buf, uint32_t len);

uint32_t gl_spi_write_and_or_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint32_t len_to_write, uint8_t *buf_to_read, uint32_t len_to_read);
uint32_t gl_spi_write_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint8_t *buf_to_read, uint32_t len);

/* SPI 2 */

void gl_spi2_init(uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin, uint8_t mode);

void gl_spi2_prepare_ss_pin(uint8_t ss_pin);

uint8_t gl_spi2_read_reg_byte(uint8_t ss_pin, uint8_t reg);
void gl_spi2_write_reg_byte(uint8_t ss_pin, uint8_t reg, uint8_t value);

uint32_t gl_spi2_read_reg(uint8_t ss_pin, uint8_t reg, uint8_t *buf, uint32_t len);
uint32_t gl_spi2_write_reg(uint8_t ss_pin, uint8_t reg, const uint8_t *buf, uint32_t len);

uint32_t gl_spi2_write_and_or_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint32_t len_to_write, uint8_t *buf_to_read, uint32_t len_to_read);
uint32_t gl_spi2_write_read(uint8_t ss_pin, const uint8_t *buf_to_write, uint8_t *buf_to_read, uint32_t len);

/* QSPI */

void gl_qspi_init(uint8_t sck_pin, uint8_t csn_pin, uint8_t io0_pin, uint8_t io1_pin, uint8_t io2_pin, uint8_t io3_pin);

uint32_t gl_qspi_erase(void);
uint32_t gl_qspi_write(uint32_t addr, const uint8_t *buf, uint32_t len);
uint32_t gl_qspi_read(uint32_t addr, uint8_t *buf, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif // GL_SPI_H__
