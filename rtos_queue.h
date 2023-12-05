#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        LCD_COMMAND_CPU_TEMPERATUE,
        LCD_COMMAND_CPU_UTILIZATION_RATE,
        LCD_COMMAND_RAM_UTILIZATION_RATE,
        LCD_COMMAND_RAM_UESD,
        LCD_COMMAND_RAM_TOTAL,
        LCD_COMMAND_GPU_TEMPERATUE,
        LCD_COMMAND_GPU_UTILIZATION_RATE,
        LCD_COMMAND_GPU_RAM_UTILIZATION_RATE,
        LCD_COMMAND_GPU_RAM_UESD,
        LCD_COMMAND_GPU_RAM_TOTAL,
    } LCD_COMMAND;

    typedef struct
    {
        LCD_COMMAND command;
        uint8_t *data;
    } config_lcd;

    void queue_init();
    void x_queue_lcd_send(LCD_COMMAND command, uint8_t *data);

    extern QueueHandle_t x_queue_lcd;

#ifdef __cplusplus
}
#endif