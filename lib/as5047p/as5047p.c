#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#include "as5047p.h"

int as5047p_init(as5047p_t* dev, const struct device* spi, uint16_t slave,
        const struct device* gpio_dev, gpio_pin_t pin) {
    dev->spi = spi;
    dev->cfg = SPI_AS5047P_CONFIG(slave, &dev->cs_ctrl);
    dev->cs_ctrl = SPI_AS5047P_CS_CTRL(gpio_dev, pin);
    return 0;
}

int as5047p_read(const as5047p_t* dev, uint16_t* rval) {
    uint16_t rx_data[1];
    const struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 1
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };
    int ret = spi_read(dev->spi, &dev->cfg, &rx);
    *rval = rx_data[0] << 2;
    return ret;
}

int as5047p_transeive(const as5047p_t* dev, uint16_t tval, uint16_t* rval) {
    uint16_t rx_data[1];
    const struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = 1
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };
    uint16_t tx_data[] = { tval };
    const struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = 1
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };
    int ret = spi_transceive(dev->spi, &dev->cfg, &tx, &rx);
    *rval = rx_data[0] << 2;
    return ret;
}

