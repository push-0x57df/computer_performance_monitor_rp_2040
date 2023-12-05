#include "rtos_queue.h"

QueueHandle_t x_queue_lcd;

void queue_init()
{
    x_queue_lcd = xQueueCreate(10, sizeof(config_lcd *));
}

config_lcd config_lcd_data;

/**
 * @brief lcd屏幕消息发送
 *
 * @param command 指令
 * @param data 消息
 */
void x_queue_lcd_send(LCD_COMMAND command, uint8_t *data)
{
    config_lcd_data.command = command;
    config_lcd_data.data = data;
    config_lcd *config_lcd_data_p;
    config_lcd_data_p = &config_lcd_data;
    xQueueSend(x_queue_lcd, (void *)&config_lcd_data_p, 50);
}
