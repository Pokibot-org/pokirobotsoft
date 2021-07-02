#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include <logging/log.h>
#include <pinmux/stm32/pinmux_stm32.h>

#include "as5047p.h"

LOG_MODULE_REGISTER(encoders);

int as5047p_init(as5047p* encoder, const struct device* spi) {
    encoder->cs_ctrl = {
        .delay = 0,
        .gpio_dt_flags = GPIO_ACTIVE_LOW,
        .gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpioa)),
        .gpio_pin = 15
    };
    encoder->cfg = {
        .frequency = 500000U,
        .operation = SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_WORD_SET(16),
        .slave = 0,
        .cs = &encoder->cs_ctrl
    };
    encoder->spi = spi;
    return 0;
}

int as5047p_read(const as5047p* dev, uint16_t* val) {
    uint16_t rx_data[1];
    const struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 1
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };
    uint16_t tx_data[] = { 0x7fff };
    const struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 1
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    //int ret = spi_transceive(dev->spi, &dev->cfg, &tx, &rx);
    int ret = spi_read(dev->spi, &dev->cfg, &rx);
    *val = rx_data[0];

    return ret;
}

