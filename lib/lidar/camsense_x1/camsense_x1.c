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

// DEFINES  
#define CAMSENSE_X1_FRAME_SIZE 36
#define CAMSENSE_X1_SPEED_L_INDEX 0
#define CAMSENSE_X1_SPEED_H_INDEX 1
#define CAMSENSE_X1_START_ANGLE_L_INDEX 2
#define CAMSENSE_X1_START_ANGLE_H_INDEX 3
#define CAMSENSE_X1_FIRST_POINT_INDEX 4
#define CAMSENSE_X1_POINT_DISTANCE_L_RELATIVE_INDEX 0
#define CAMSENSE_X1_POINT_DISTANCE_H_RELATIVE_INDEX 1
#define CAMSENSE_X1_POINT_QUALITY_RELATIVE_INDEX 2
#define CAMSENSE_X1_END_ANGLE_L_INDEX 28
#define CAMSENSE_X1_END_ANGLE_H_INDEX 29

typedef enum
{
    frame_parsing_state_header_sync,
    frame_parsing_state_normal
} Frame_parsing_state;

// PRIVATE VARIABLE
static const struct device *uart_dev;
static const uint8_t camsense_x1_header[] = {0x55, 0xAA, 0x03, 0x08};
#define CAMSENSE_X1_HEADER_SIZE ARRAY_SIZE(camsense_x1_header)

K_MSGQ_DEFINE(lidar_msgq, sizeof(lidar_message_t), 10, 1);

static uint8_t frame_buffer[CAMSENSE_X1_FRAME_SIZE - CAMSENSE_X1_HEADER_SIZE];

static float camsense_x1_speed = 0;
// PRIVATE FUNC
void process_recived_frame(uint8_t *recived_frame)
{
    LOG_DBG("Receiving lidar frame");
    camsense_x1_speed = ((uint16_t)(recived_frame[CAMSENSE_X1_SPEED_H_INDEX] << 8) | recived_frame[CAMSENSE_X1_SPEED_L_INDEX]) / 3840.0; // 3840.0 = (64 * 60)
    lidar_message_t message = {0};
    message.start_angle = (((uint16_t)recived_frame[CAMSENSE_X1_START_ANGLE_H_INDEX]) << 8 | recived_frame[CAMSENSE_X1_START_ANGLE_L_INDEX]) / 64.0 - 640.0; // TODO: Use shift not /
    message.end_angle = (((uint16_t)recived_frame[CAMSENSE_X1_END_ANGLE_H_INDEX]) << 8 | recived_frame[CAMSENSE_X1_END_ANGLE_L_INDEX]) / 64.0 - 640.0;

    for (int point_index = 0; point_index < LIDAR_MESSAGE_NUMBER_OF_POINT; point_index++) // for each of the 8 samples
    {

        uint8_t distance_l = recived_frame[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_DISTANCE_L_RELATIVE_INDEX + (point_index * 3)];
        uint8_t distance_h = recived_frame[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_DISTANCE_H_RELATIVE_INDEX + (point_index * 3)];
        uint8_t quality = recived_frame[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_QUALITY_RELATIVE_INDEX + (point_index * 3)];

        message.points[point_index].distance = (((uint16_t)distance_h) << 8) | (uint16_t)distance_l;
        message.points[point_index].quality = quality;
    }
    LOG_DBG("Putting lidar message");
    while(k_msgq_put(&lidar_msgq, &message, K_NO_WAIT)){
        k_msgq_purge(&lidar_msgq);
        LOG_ERR("Queue to full, purging");
    }
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
            if (header_sync_index == CAMSENSE_X1_HEADER_SIZE)
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
        if (frame_index == (CAMSENSE_X1_FRAME_SIZE - CAMSENSE_X1_HEADER_SIZE - 1))
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

float camsense_x1_get_sensor_speed()
{
    return camsense_x1_speed;
}

void camsense_x1_read_sensor(lidar_message_t *message){
    k_msgq_get(&lidar_msgq, message, K_FOREVER);
    LOG_DBG("Reading message");
}