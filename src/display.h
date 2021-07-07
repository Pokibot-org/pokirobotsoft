#ifndef DISPLAY_H
#define DISPLAY_H

#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#define SPI_DISPLAY_FREQ        200000U
#define SPI_DISPLAY_OP          (SPI_OP_MODE_MASTER | \
                                 SPI_TRANSFER_MSB | SPI_WORD_SET(8))
#define SPI_DISPLAY_CS_FLAGS    GPIO_ACTIVE_LOW
#define SPI_DISPLAY_CS_DELAY    0

#define USER_DATA_SIZE 16
#define SCORE_DIGIT 3

#define DISPLAY_SPI_DEV        DT_LABEL(DT_ALIAS(spi_display))
#define DISPLAY_CS_GPIO_DEV    DEVICE_DT_GET(DT_NODELABEL(gpiob))
#define DISPLAY_CS_PIN     1


#define SPI_DISPLAY_CONFIG(_slave, _cs_ctrl)                                    \
    (struct spi_config) {                                                       \
        .frequency = SPI_DISPLAY_FREQ,                                          \
        .operation = SPI_DISPLAY_OP,                                            \
        .slave = (_slave),                                                      \
        .cs = (_cs_ctrl)                                                        \
    }

#define SPI_DISPLAY_CS_CTRL(_gpio_dev, _pin)                                    \
    (struct spi_cs_control) {                                                   \
        .gpio_dev = (_gpio_dev),                                                \
        .delay = SPI_DISPLAY_CS_DELAY,                                          \
        .gpio_pin = (_pin),                                                     \
        .gpio_dt_flags = SPI_DISPLAY_CS_FLAGS                                   \
    }



typedef struct DISPLAY
{
	const struct device* spi;
	struct spi_config cfg;
	struct spi_cs_control cs_ctrl;
} display_t;


int display_init();
int display_send(int score);

#endif // DISPLAY_H
