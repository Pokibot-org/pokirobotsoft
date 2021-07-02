#include <zephyr.h>
#include <logging/log.h>
#include <drivers/spi.h>

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
    as5047p_init(&enc_l, spi);

    while (1)
    {
        k_sleep(K_MSEC(1000));
        uint16_t val = 1;
        //int ret = 0;
        int ret = as5047p_read(&enc_l, &val);
        LOG_INF("transceive (ret=%d): %x", ret, val & 0xFFFF);
    }

    return 0;

}
