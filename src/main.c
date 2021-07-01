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
            LOG_INF("fail device_get\n");
        }
    }

    as5047p enc_left;
    as5047p_init(&enc_left, spi);

    while (1)
    {
        k_sleep(K_MSEC(1000));
        uint32_t val;
        //int ret = 0;
        int ret = as5047p_read(&enc_left, &val);
        if (!ret) {
            LOG_INF("success read\n");
        } else {
            LOG_INF("fail read\n");
        }
    }

    return 0;

}
