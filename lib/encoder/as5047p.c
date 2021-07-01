#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include <logging/log.h>

#include "as5047p.h"

LOG_MODULE_REGISTER(encoders);

int as5047p_init(as5047p* encoder, const struct device* spi) {
    encoder->cfg.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_WORD_SET(8);
    encoder->cfg.frequency = 100000U;
    return 0;
}

int as5047p_read(const as5047p* dev, uint32_t* val) {
    uint32_t data[4];
    const struct spi_buf buf = {
        .buf = data,
        .len = 4
    };
    const struct spi_buf_set rx = {
        .buffers = &buf,
        .count = 4
    };

    LOG_INF("aaaa");
    int read = spi_read(dev->spi, &dev->cfg, &rx);
    LOG_INF("bbbb");
    *val = data[0];

    return read;
}

