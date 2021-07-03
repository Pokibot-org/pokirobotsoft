
#include <devicetree.h>
#include <drivers/spi.h>

// spi for as5047p magnetic encoders
#define SPI_AS5047P_FREQ        500000U
#define SPI_AS5047P_OP          (SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_TRANSFER_MSB | SPI_WORD_SET(16))
#define SPI_AS5047P_CS_FLAGS    GPIO_ACTIVE_LOW

