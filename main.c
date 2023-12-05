#include "FreeRTOS.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "task.h"
#include "rtos_task.h"
#include "pico/stdio_usb.h"
#include "queue.h"
#include "rtos_queue.h"

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

int main()
{
    stdio_init_all();

    queue_init();

    xTaskCreate(v_task_lcd_Init, "lcd_init", 1000, NULL, 31, NULL);
    xTaskCreate(v_task_usb_uart, "usb_uart", 500, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1)
    {
    }
}