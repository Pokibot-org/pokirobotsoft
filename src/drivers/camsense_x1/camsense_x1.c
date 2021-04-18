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

K_MSGQ_DEFINE(lidar_msgq, sizeof(lidar_message_t), 80, 1);

typedef struct camsense_x1_obj
{
    uint8_t frame_buffer[CAMSENSE_X1_FRAME_SIZE];
    float current_speed;
    camsense_x1_on_rotation_clbk on_rotation_callback;
} camsense_x1_obj_t;

camsense_x1_obj_t obj = {0};

// PRIVATE FUNC

void process_recived_frame(uint8_t *recived_frame_no_resync_header)
{
    static float previous_angle = 0;
    LOG_DBG("Receiving lidar frame");
    obj.current_speed = ((uint16_t)(recived_frame_no_resync_header[CAMSENSE_X1_SPEED_H_INDEX] << 8) | recived_frame_no_resync_header[CAMSENSE_X1_SPEED_L_INDEX]) / 3840.0; // 3840 = (64 * 60)
    lidar_message_t message = {0};
    message.start_angle = (((uint16_t)recived_frame_no_resync_header[CAMSENSE_X1_START_ANGLE_H_INDEX]) << 8 | recived_frame_no_resync_header[CAMSENSE_X1_START_ANGLE_L_INDEX]) / 64.0 - 640; // TODO: Use shift not /
    message.end_angle = (((uint16_t)recived_frame_no_resync_header[CAMSENSE_X1_END_ANGLE_H_INDEX]) << 8 | recived_frame_no_resync_header[CAMSENSE_X1_END_ANGLE_L_INDEX]) / 64.0 - 640;

    for (int point_index = 0; point_index < LIDAR_MESSAGE_NUMBER_OF_POINT; point_index++) // for each of the 8 samples
    {

        uint8_t distance_l = recived_frame_no_resync_header[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_DISTANCE_L_RELATIVE_INDEX + (point_index * 3)];
        uint8_t distance_h = recived_frame_no_resync_header[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_DISTANCE_H_RELATIVE_INDEX + (point_index * 3)];
        uint8_t quality = recived_frame_no_resync_header[CAMSENSE_X1_FIRST_POINT_INDEX + CAMSENSE_X1_POINT_QUALITY_RELATIVE_INDEX + (point_index * 3)];

        message.points[point_index].distance = (((uint16_t)distance_h) << 8) | (uint16_t)distance_l;
        message.points[point_index].quality = quality;
    }

    while (k_msgq_put(&lidar_msgq, &message, K_NO_WAIT))
    {
        k_msgq_purge(&lidar_msgq);
        LOG_ERR("Queue to full, purging");
    }

    // If one rotation happend, call the on_rotation_callbackon_rotation_callback callback
    if (previous_angle <= 180 && message.start_angle >= 180)
    {
        if (obj.on_rotation_callback)
        {
            obj.on_rotation_callback();
        }
    }
    previous_angle = message.start_angle;
}

void camsense_x1_read_one_frame()
{
    int err = uart_rx_enable(uart_dev, obj.frame_buffer, CAMSENSE_X1_HEADER_SIZE, SYS_FOREVER_MS);
    if (err)
    {
        LOG_ERR("Cant start full frame read");
    }
}

void camsense_x1_full_frame_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    if (evt->type == UART_RX_RDY)
    {
        uint8_t *frame = evt->data.rx.buf;
        if (memcmp(frame, camsense_x1_header, CAMSENSE_X1_HEADER_SIZE) == 0)
        {
            process_recived_frame(&frame[4]);
        }
        else
        {
            // Resync
            LOG_WRN("Wrong header, resync started");
            uart_irq_rx_enable(uart_dev);
            return;
        }
        
        // Reload dma
        camsense_x1_read_one_frame();
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
        obj.frame_buffer[frame_index] = recived_byte;
        frame_index += 1;
        if (frame_index == (CAMSENSE_X1_FRAME_SIZE - CAMSENSE_X1_HEADER_SIZE - 1))
        {
            frame_index = 0;
            parsing_state = frame_parsing_state_header_sync;
            process_recived_frame(obj.frame_buffer);
#if CONFIG_UART_ASYNC_API
            uart_irq_rx_disable(uart_dev);
            camsense_x1_read_one_frame();
#endif
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
        // CONFIG UART
        uart_dev = device_get_binding(DT_LABEL(DT_ALIAS(camsense_uart_1)));
        if (uart_dev == NULL)
        {
            LOG_ERR("Cant get the uart device binding");
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

#if CONFIG_UART_ASYNC_API
        // CONFIG UART DMA
        err = uart_callback_set(uart_dev, camsense_x1_full_frame_callback, &obj);
        if (err)
        {
            LOG_ERR("Error in uart_callback_set, ASYNC API not supported ? err %d", err);
        }
#endif
        // START DRIVER
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
    return obj.current_speed;
}

int camsense_x1_read_sensor_blocking(lidar_message_t *message)
{
    return k_msgq_get(&lidar_msgq, message, K_FOREVER);
}

int camsense_x1_read_sensor_non_blocking(lidar_message_t *message)
{
    return k_msgq_get(&lidar_msgq, message, K_NO_WAIT);
}

void camsense_x1_add_on_rotation_clbk(camsense_x1_on_rotation_clbk fun)
{
    obj.on_rotation_callback = fun;
}
