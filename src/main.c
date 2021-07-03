#include <zephyr.h>
#include <logging/log.h>
#include <drivers/spi.h>
#include <devicetree.h>
#include <device.h>

#include "pathfinding_test.h"
#include "as5047p.h"

LOG_MODULE_REGISTER(main_task);


int main(void)
{

    LOG_INF("boot\n");

    //pathfinding_test_main();
    const struct device* spi = device_get_binding(DT_LABEL(DT_ALIAS(enc_left)));
    if (!spi) {
        LOG_ERR("failed to get SPI");
        while (1)
        {
            k_sleep(K_MSEC(1000));
            LOG_INF("fail device_get");
        }
    }

    as5047p enc_l;
    as5047p_init(&enc_l, spi, 0, DEVICE_DT_GET(DT_NODELABEL(gpioa)), 4);
    //as5047p_init(&enc_l, spi);


    //int a = DT_SPI_DEV_HAS_CS_GPIOS(DT_NODELABEL(as5047p_r));
    //DT_NODELABEL(as5047p_r);
    //struct spi_cs_control *cs_ctrl = SPI_CS_CONTROL_PTR_DT(DT_NODELABEL(as5047p_r), 0);
    //LOG_INF("%d", a);

    while (1)
    {
        k_sleep(K_MSEC(1000));
        uint16_t val = 1;
        //int ret = 0;
        int ret = as5047p_read(&enc_l, &val);
        LOG_INF("transceive (ret=%d): par=%x err=%x  ---  %d", ret, (val & (1<<15)) >> 15, (val & (1<<14)) >> 14, val & 0x3fff);
    }

    return 0;

}
