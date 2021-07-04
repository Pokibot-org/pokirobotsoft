#include <zephyr.h>
#include <logging/log.h>
#include <drivers/spi.h>
#include <devicetree.h>
#include <device.h>

#include "as5047p.h"

LOG_MODULE_REGISTER(main);


int main(void) {

    LOG_INF("boot\n");

    const struct device* spi = device_get_binding(
            DT_LABEL(DT_ALIAS(spi_as5047p)));
    if (!spi) {
        LOG_ERR("failed to get SPI");
        while (1) {
            k_sleep(K_MSEC(1000));
            LOG_INF("fail device_get");
        }
    }

    as5047p enc_l;
    as5047p enc_r;
    as5047p_init(&enc_l, spi, 0, DEVICE_DT_GET(DT_NODELABEL(gpioa)), 4);
    as5047p_init(&enc_r, spi, 0, DEVICE_DT_GET(DT_NODELABEL(gpioa)), 15);

    while (1) {
        uint16_t val_l, val_r;
        k_sleep(K_MSEC(1000));
        int ret_l = as5047p_read(&enc_l, &val_l);
        int ret_r = as5047p_read(&enc_r, &val_r);
        LOG_INF("read_r (ret=%d): par=%x err=%x  ---  %d",
                ret_l,
                (val_l & (1<<15)) >> 15,
                (val_l & (1<<14)) >> 14, val_l & 0x3fff);
        LOG_INF("read_r (ret=%d): par=%x err=%x  ---  %d",
                ret_r,
                (val_r & (1<<15)) >> 15,
                (val_r & (1<<14)) >> 14, val_r & 0x3fff);
    }

    return 0;

}
