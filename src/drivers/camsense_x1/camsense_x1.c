#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <drivers/uart.h>

#include "camsense_x1.h"

/*
Format of frames:
    <0x55><0xAA><0x03><0x08>
    <speedL><speedH>
    <startAngleL><startAngleH>
    <distance0L><distance0H><quality0>
    <distance1L><distance1H><quality1>
    <distance2L><distance2H><quality2>
    <distance3L><distance3H><quality3>
    <distance4L><distance4H><quality4>
    <distance5L><distance5H><quality5>
    <distance6L><distance6H><quality6>
    <distance7L><distance7H><quality7>
    <endAngleL><endAngleH>
    <unknown><unknown> could be a CRC

    A package always starts with <0x55><0xAA><0x03><0x08>
*/

LOG_MODULE_REGISTER(camsense_x1_driver);

// DEFINES PRE VARIABLES
#define CAMSENSE_X1_FRAME_SIZE 36

typedef enum
{
    frame_parsing_state_header_sync,
    frame_parsing_state_normal
} Frame_parsing_state;

// PRIVATE VARIABLE
static const struct device *uart_dev;
static const uint8_t camsense_x1_header[] = {0x55, 0xAA, 0x03, 0x08};
#define CAMSENSE_X1_HEADER_SIZE ARRAY_SIZE(camsense_x1_header)

static uint8_t frame_buffer[CAMSENSE_X1_FRAME_SIZE - CAMSENSE_X1_HEADER_SIZE];

// PRIVATE FUNC
void process_recived_frame(uint8_t *recived_frame)
{
    
}

void uart_rx_callback(const struct device *dev, void *user_data)
{
    // No need to test uart_irq_rx_ready() if the only it trigger enabeled is rx
    uint8_t recived_byte;
    static uint8_t frame_index = 0;
    static uint8_t header_sync_index = 0;
    static Frame_parsing_state parsing_state = frame_parsing_state_header_sync;

    uart_fifo_read(dev, &recived_byte, 1);

    switch (parsing_state)
    {
    case frame_parsing_state_header_sync:

        if (recived_byte == camsense_x1_header[header_sync_index])
        {
            header_sync_index += 1;
            if (header_sync_index == (CAMSENSE_X1_HEADER_SIZE - 1))
            {
                header_sync_index = 0;
                parsing_state = frame_parsing_state_normal;
            }
        }
        else
        {
            header_sync_index = 0;
        }

        break;
    case frame_parsing_state_normal:
        frame_buffer[frame_index] = recived_byte;
        frame_index += 1;
        if (frame_index == (CAMSENSE_X1_FRAME_SIZE - 1))
        {
            frame_index = 0;
            parsing_state = frame_parsing_state_header_sync;
            process_recived_frame(frame_buffer);
        }
        break;
    default:
        LOG_ERR("Not supposed to be here, state %d", parsing_state);
        break;
    }
}

// PUBLIC FUNC

/** 
 * @brief Camsense driver init 
 * 
 *  @return -1 if error in the init, -2 if already init, of otherwise
 */
int camsense_x1_init()
{
    if (uart_dev == NULL)
    {
        uart_dev = device_get_binding(DT_LABEL(DT_ALIAS(camsense_uart_1)));
        if (uart_dev == NULL)
        {
            LOG_ERR("Cant get the device binding");
            return -1;
        }
        const struct uart_config cfg = {
            .baudrate = 115200,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1};
        int err = uart_configure(uart_dev, &cfg);
        if (err)
        {
            LOG_ERR("Cant configure the uart");
            return -1;
        }

        uart_irq_callback_set(uart_dev, uart_rx_callback);
        uart_irq_rx_enable(uart_dev);
        return 0;
    }
    else
    {
        LOG_WRN("camsense_x1_init already called");
    }

    LOG_INF("Camsense init done!");
    return -2;
}