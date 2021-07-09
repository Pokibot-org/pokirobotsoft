#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

#include "display.h"

display_t dev_display;
const struct device* spi_display;

static unsigned char _seven_seg[10] = {
		0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7C,0x07,0x7F,0x67};

int display_spi_init(display_t* dev, const struct device* spi, uint16_t slave,
					 const struct device* gpio_dev, gpio_pin_t pin) {
	dev->spi = spi;
	dev->cfg = SPI_DISPLAY_CONFIG(slave, &dev->cs_ctrl);
	dev->cs_ctrl = SPI_DISPLAY_CS_CTRL(gpio_dev, pin);
	return 0;
}

int display_init(){

	spi_display = device_get_binding(DISPLAY_SPI_DEV);
	if (!spi_display) {
		return -1;
	}
	int err = display_spi_init(&dev_display, spi_display, 0, DISPLAY_CS_GPIO_DEV, DISPLAY_CS_PIN);
	if (err)
	{
		LOG_ERR("Problem with the spi init");
	}
}




int display_transeive(const display_t* dev, uint16_t tval, uint16_t* rval) {
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

int display_send(int score){
	int i;
	int j;
	unsigned char digit[SCORE_DIGIT];
	int val_dis;

	for (j = 0; j < SCORE_DIGIT; j++){
		digit[j] = score % 10;
		score /= 10;
	}

	for (j = 0; j < SCORE_DIGIT; j++){
		display_transeive(&dev_display, ~_seven_seg[digit[j]] , &val_dis);
	}
}

