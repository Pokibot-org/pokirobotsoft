#ifndef AS5047P_H
#define AS5047P_H

#include <zephyr.h>
#include <drivers/spi.h>

typedef struct as5047p
{
    const struct device* spi;
    struct spi_config cfg;
} as5047p;

int as5047p_init(as5047p* encoder, const struct device* spi);
int as5047p_read(const as5047p* encoder, uint32_t* val);

#endif // AS5047P_H
