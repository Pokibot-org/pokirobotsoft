#ifndef AS5047P_H
#define AS5047P_H

#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#define SPI_AS5047P_FREQ        1000000U
#define SPI_AS5047P_OP          (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | \
                                 SPI_TRANSFER_MSB | SPI_WORD_SET(16))
#define SPI_AS5047P_CS_FLAGS    GPIO_ACTIVE_LOW
#define SPI_AS5047P_CS_DELAY    0


#define SPI_AS5047P_CONFIG(_slave, _cs_ctrl)                                    \
    (struct spi_config) {                                                       \
        .frequency = SPI_AS5047P_FREQ,                                          \
        .operation = SPI_AS5047P_OP,                                            \
        .slave = (_slave),                                                      \
        .cs = (_cs_ctrl)                                                        \
    }

#define SPI_AS5047P_CS_CTRL(_gpio_dev, _pin)                                    \
    (struct spi_cs_control) {                                                   \
        .gpio_dev = (_gpio_dev),                                                \
        .delay = SPI_AS5047P_CS_DELAY,                                          \
        .gpio_pin = (_pin),                                                     \
        .gpio_dt_flags = SPI_AS5047P_CS_FLAGS                                   \
    }


typedef struct as5047p
{
    const struct device* spi;
    struct spi_config cfg;
    struct spi_cs_control cs_ctrl;
} as5047p_t;


int as5047p_init(as5047p_t* dev, const struct device* spi, uint16_t slave,
        const struct device* gpio_dev, gpio_pin_t pin);
int as5047p_read(const as5047p_t* dev, uint16_t* rval);
int as5047p_transeive(const as5047p_t* dev, uint16_t tval, uint16_t* rval);

#endif // AS5047P_H
