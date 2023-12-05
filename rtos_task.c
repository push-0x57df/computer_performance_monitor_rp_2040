#include "FreeRTOS.h"
#include "rtos_task.h"
#include "task.h"
#include "lvgl-8.3.10/lvgl.h"
#include "hardware/gpio.h"
#include "io_config.h"
#include "hardware/pio.h"
#include "i8080.pio.h"
#include "hardware/dma.h"
#include "stdlib.h"
#include "lvgl-8.3.10/demos/lv_demos.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "rtos_queue.h"

#define DISP_HOR_RES 320
#define DISP_VER_RES 170

uint dma_channel;
dma_channel_config dma_config;
PIO pio;
uint sm;
extern lv_font_t HarmonyOS_2bit;

void lcd_bus_wait()
{
    while (dma_channel_is_busy(dma_channel))
    {
        vTaskDelay(1);
    }
}

void lcd_wr_dat(uint8_t *dat, uint len)
{
    lcd_bus_wait();
    dma_channel_configure(
        dma_channel,
        &dma_config,
        &pio->txf[sm],
        dat,
        len,
        true);
}

void lcd_wr_reg(uint8_t *dat, uint len)
{
    lcd_bus_wait();
    gpio_put(LCD_PIN_NUM_DC, 0);
    lcd_wr_dat(dat, len);
    lcd_bus_wait();
    gpio_put(LCD_PIN_NUM_DC, 1);
}

void lcd_wr_dat8(uint8_t dat)
{
    uint8_t *buf = malloc(sizeof(uint8_t));
    *buf = dat;
    lcd_wr_dat(buf, 1);
    free(buf);
}

void lcd_wr_dat16(uint16_t dat)
{
    lcd_wr_dat8(dat >> 8);
    lcd_wr_dat8(dat);
}

void lcd_wr_reg8(uint8_t dat)
{
    uint8_t *buf = malloc(sizeof(uint8_t));
    *buf = dat;
    lcd_wr_reg(buf, 1);
    free(buf);
}

void vTaskDelay_ms(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void lcd_init()
{
    gpio_put(LCD_PIN_NUM_BK_LIGHT, 0);
    gpio_put(LCD_PIN_NUM_DC, 1);

    // 重置动作
    gpio_put(LCD_PIN_NUM_RST, 0);
    vTaskDelay_ms(100);
    gpio_put(LCD_PIN_NUM_RST, 1);

    // 初始化指令和数据
    lcd_wr_reg8(0x11);

    vTaskDelay_ms(120);

    lcd_wr_reg8(0x36);
    lcd_wr_dat8(0xA0);

    lcd_wr_reg8(0x3A);
    lcd_wr_dat8(0x05);

    lcd_wr_reg8(0xB2);
    lcd_wr_dat8(0x0C);
    lcd_wr_dat8(0x0C);
    lcd_wr_dat8(0x00);
    lcd_wr_dat8(0x33);
    lcd_wr_dat8(0x33);

    lcd_wr_reg8(0xB7);
    lcd_wr_dat8(0x35);

    lcd_wr_reg8(0xBB);
    lcd_wr_dat8(0x1A);

    lcd_wr_reg8(0xC0);
    lcd_wr_dat8(0x2C);

    lcd_wr_reg8(0xC2);
    lcd_wr_dat8(0x01);

    lcd_wr_reg8(0xC3);
    lcd_wr_dat8(0x0B);

    lcd_wr_reg8(0xC4);
    lcd_wr_dat8(0x20);

    lcd_wr_reg8(0xC6);
    lcd_wr_dat8(0x0F);

    lcd_wr_reg8(0xD0);
    lcd_wr_dat8(0xA4);
    lcd_wr_dat8(0xA1);

    lcd_wr_reg8(0x21);
    lcd_wr_reg8(0xE0);
    lcd_wr_dat8(0xF0);
    lcd_wr_dat8(0x00);
    lcd_wr_dat8(0x04);
    lcd_wr_dat8(0x04);
    lcd_wr_dat8(0x04);
    lcd_wr_dat8(0x05);
    lcd_wr_dat8(0x29);
    lcd_wr_dat8(0x33);
    lcd_wr_dat8(0x3E);
    lcd_wr_dat8(0x38);
    lcd_wr_dat8(0x12);
    lcd_wr_dat8(0x12);
    lcd_wr_dat8(0x28);
    lcd_wr_dat8(0x30);

    lcd_wr_reg8(0xE1);
    lcd_wr_dat8(0xF0);
    lcd_wr_dat8(0x07);
    lcd_wr_dat8(0x0A);
    lcd_wr_dat8(0x0D);
    lcd_wr_dat8(0x0B);
    lcd_wr_dat8(0x07);
    lcd_wr_dat8(0x28);
    lcd_wr_dat8(0x33);
    lcd_wr_dat8(0x3E);
    lcd_wr_dat8(0x36);
    lcd_wr_dat8(0x14);
    lcd_wr_dat8(0x14);
    lcd_wr_dat8(0x29);
    lcd_wr_dat8(0x32);

    lcd_wr_reg8(0x11);
    vTaskDelay_ms(120);
    lcd_wr_reg8(0x29);
}

void lcd_addr_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    lcd_wr_reg8(0x2a);
    lcd_wr_dat16(x1);
    lcd_wr_dat16(x2);
    lcd_wr_reg8(0x2b);
    lcd_wr_dat16(y1 + 35);
    lcd_wr_dat16(y2 + 35);
    lcd_wr_reg8(0x2c);
}

void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    lcd_addr_set(area->x1, area->y1, area->x2, area->y2);
    uint i = 0;
    lcd_wr_dat((uint8_t *)color_p, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2);
    lcd_bus_wait();
    lv_disp_flush_ready(disp); /* Indicate you are ready with the flushing*/
}

bool lv_tick_timer_callback(struct repeating_timer *t)
{
    lv_tick_inc(5);
    return true;
}

// 计算进度条颜色
uint32_t calc_color(uint8_t rate)
{
    if (rate > 100) rate = 100;  // 限制rate的范围

    uint8_t red = (255 * rate) / 100;
    uint8_t green = 255 - red;
    uint8_t blue = 0;

    return (red << 16) | (green << 8) | blue;
}

void v_task_lcd_Init(void *pvParameters)
{
    // 初始LCD屏幕
    uint io_mask = (1u << LCD_PIN_NUM_RD |
                    1u << LCD_PIN_NUM_BK_LIGHT |
                    1u << LCD_PIN_NUM_RST |
                    1u << LCD_PIN_NUM_DC |
                    1u << LCD_PIN_NUM_CS |
                    1u << LCD_PIN_NUM_PCLK |
                    1u << LCD_PIN_NUM_DATA7 |
                    1u << LCD_PIN_NUM_DATA6 |
                    1u << LCD_PIN_NUM_DATA5 |
                    1u << LCD_PIN_NUM_DATA4 |
                    1u << LCD_PIN_NUM_DATA3 |
                    1u << LCD_PIN_NUM_DATA2 |
                    1u << LCD_PIN_NUM_DATA1 |
                    1u << LCD_PIN_NUM_DATA0);
    gpio_init_mask(io_mask);
    gpio_set_dir_out_masked(io_mask);
    // 创建PIO实例
    pio = pio0;
    uint offset = pio_add_program(pio, &i8080_program);
    sm = pio_claim_unused_sm(pio, true);
    i8080_program_init(pio, sm, offset, LCD_PIN_NUM_DATA0, 8, LCD_PIN_NUM_CS, 2);

    // 实现pio对应的dma
    dma_channel = 0;
    dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(pio, sm, true));
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);

    // 拉高LCD_PIN_NUM_RD信号，禁止从st7789数据读出
    gpio_put(LCD_PIN_NUM_RD, 1);

    uint8_t *buf = malloc(sizeof(uint8_t));
    *buf = 0x00;
    dma_channel_configure(
        dma_channel,
        &dma_config,
        &pio->txf[sm],
        buf,
        1,
        true);

    // 开始屏幕初始化配置
    lcd_init();

    lv_init();
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[DISP_HOR_RES * DISP_VER_RES / 5];                        /*Declare a buffer for 1/10 screen size*/
    static lv_color_t buf2[DISP_HOR_RES * DISP_VER_RES / 5];                        /*Declare a buffer for 1/10 screen size*/
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, DISP_HOR_RES * DISP_VER_RES / 10); /*Initialize the display buffer.*/

    static lv_disp_drv_t disp_drv; /*Descriptor of a display driver*/

    lv_disp_drv_init(&disp_drv);     /*Basic initialization*/
    disp_drv.flush_cb = disp_flush;  /*Set your driver function*/
    disp_drv.draw_buf = &draw_buf;   /*Assign the buffer to the display*/
    disp_drv.hor_res = DISP_HOR_RES; /*Set the horizontal resolution of the display*/
    disp_drv.ver_res = DISP_VER_RES; /*Set the vertical resolution of the display*/
    lv_disp_drv_register(&disp_drv); /*Finally register the driver*/

    struct repeating_timer lv_tick_timer;
    add_repeating_timer_ms(5, lv_tick_timer_callback, NULL, &lv_tick_timer);

    gpio_put(LCD_PIN_NUM_BK_LIGHT, 1);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());  // 创建一个容器对象
    lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES); // 设置容器尺寸为屏幕尺寸
    lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_style_t cont_style;
    lv_style_init(&cont_style);
    lv_style_set_bg_color(&cont_style, lv_color_hex(0x1b1b1b));
    lv_style_set_border_width(&cont_style, 0);
    lv_style_set_radius(&cont_style, 0);
    lv_style_set_pad_all(&cont_style, 4);
    lv_style_set_outline_width(&cont_style, 0);
    lv_obj_add_style(cont, &cont_style, 0);

    lv_obj_t *panel0 = lv_obj_create(cont);
    lv_obj_set_size(panel0, 304, 35);

    lv_obj_t *panel1 = lv_obj_create(cont);
    lv_obj_set_size(panel1, 304, 35);

    lv_obj_t *panel2 = lv_obj_create(cont);
    lv_obj_set_size(panel2, 304, 68);

    lv_style_t panel_style;
    lv_style_init(&panel_style);
    lv_style_set_bg_color(&panel_style, lv_color_hex(0x313131));
    lv_style_set_border_width(&panel_style, 0);
    lv_style_set_pad_all(&panel_style, 4);
    lv_style_set_outline_width(&panel_style, 0);
    lv_style_set_layout(&panel_style, LV_LAYOUT_FLEX);
    lv_style_set_pad_column(&panel_style, 4);
    lv_style_set_pad_row(&panel_style, 4);
    lv_style_set_flex_flow(&panel_style, LV_FLEX_FLOW_ROW_WRAP);
    lv_style_set_flex_main_place(&panel_style, LV_FLEX_ALIGN_SPACE_EVENLY);
    lv_style_set_flex_cross_place(&panel_style, LV_FLEX_ALIGN_CENTER);
    lv_style_set_flex_track_place(&panel_style, LV_FLEX_ALIGN_CENTER);
    // lv_style_set_radius(&panel_style, 0);
    lv_obj_add_style(panel0, &panel_style, 0);
    lv_obj_add_style(panel1, &panel_style, 0);

    lv_style_t panel_style2;
    lv_style_init(&panel_style2);
    lv_style_set_bg_color(&panel_style2, lv_color_hex(0x313131));
    lv_style_set_border_width(&panel_style2, 0);
    lv_style_set_pad_all(&panel_style2, 4);
    lv_style_set_outline_width(&panel_style2, 0);
    lv_style_set_layout(&panel_style2, LV_LAYOUT_GRID);
    lv_style_set_pad_column(&panel_style2, 4);
    lv_style_set_pad_row(&panel_style2, 4);
    static lv_coord_t panel_style2_col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t panel_style2_row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_style_set_grid_column_dsc_array(&panel_style2, panel_style2_col_dsc);
    lv_style_set_grid_row_dsc_array(&panel_style2, panel_style2_row_dsc);
    lv_obj_add_style(panel2, &panel_style2, 0);

    lv_obj_t *item0 = lv_obj_create(panel0);
    lv_obj_set_height(item0, 27);
    lv_obj_set_flex_grow(item0, 1);

    lv_obj_t *item1 = lv_obj_create(panel0);
    lv_obj_set_height(item1, 27);
    lv_obj_set_flex_grow(item1, 1);

    lv_obj_t *item2 = lv_obj_create(panel1);
    lv_obj_set_height(item2, 27);
    lv_obj_set_flex_grow(item2, 1);

    lv_obj_t *item3 = lv_obj_create(panel2);
    lv_obj_set_size(item3, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(item3, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);

    lv_obj_t *item4 = lv_obj_create(panel2);
    lv_obj_set_size(item4, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(item4, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);

    lv_obj_t *item5 = lv_obj_create(panel2);
    lv_obj_set_size(item5, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(item5, LV_GRID_ALIGN_STRETCH, 0, 2,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_style_t item_style;
    lv_style_init(&item_style);
    lv_style_set_bg_color(&item_style, lv_color_hex(0x262626));
    lv_style_set_border_width(&item_style, 0);
    lv_style_set_pad_all(&item_style, 0);
    lv_style_set_outline_width(&item_style, 0);
    lv_style_set_layout(&item_style, LV_LAYOUT_GRID);
    lv_style_set_pad_row(&item_style, 2);
    lv_style_set_pad_hor(&item_style, 4);
    lv_style_set_pad_ver(&item_style, 3);
    static lv_coord_t item_style_col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t item_style_row_dsc[] = {LV_GRID_FR(6), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_style_set_grid_column_dsc_array(&item_style, item_style_col_dsc);
    lv_style_set_grid_row_dsc_array(&item_style, item_style_row_dsc);
    lv_obj_add_style(item0, &item_style, 0);
    lv_obj_add_style(item1, &item_style, 0);
    lv_obj_add_style(item2, &item_style, 0);
    lv_obj_add_style(item3, &item_style, 0);
    lv_obj_add_style(item4, &item_style, 0);
    lv_obj_add_style(item5, &item_style, 0);

    lv_obj_t *label0 = lv_label_create(item0);
    lv_label_set_text(label0, "CPU温度");
    lv_obj_set_size(label0, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label0, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar0 = lv_bar_create(item0);
    lv_bar_set_range(bar0, 0, 100);
    lv_obj_set_size(bar0, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar0, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *label1 = lv_label_create(item1);
    lv_label_set_text(label1, "CPU利用率");
    lv_obj_set_size(label1, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label1, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar1 = lv_bar_create(item1);
    lv_bar_set_range(bar1, 0, 100);
    lv_obj_set_size(bar1, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar1, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *label2 = lv_label_create(item2);
    lv_label_set_text(label2, "RAM利用率");
    lv_obj_set_size(label2, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label2, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar2 = lv_bar_create(item2);
    lv_bar_set_range(bar2, 0, 100);
    lv_obj_set_size(bar2, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar2, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *label3 = lv_label_create(item3);
    lv_label_set_text(label3, "GPU温度");
    lv_obj_set_size(label3, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label3, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar3 = lv_bar_create(item3);
    lv_bar_set_range(bar3, 0, 100);
    lv_obj_set_size(bar3, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar3, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *label4 = lv_label_create(item4);
    lv_label_set_text(label4, "GPU利用率");
    lv_obj_set_size(label4, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label4, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar4 = lv_bar_create(item4);
    lv_bar_set_range(bar4, 0, 100);
    lv_obj_set_size(bar4, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar4, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_obj_t *label5 = lv_label_create(item5);
    lv_label_set_text(label5, "GPU内存利用率");
    lv_obj_set_size(label5, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(label5, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_START, 0, 1);

    lv_obj_t *bar5 = lv_bar_create(item5);
    lv_bar_set_range(bar5, 0, 100);
    lv_obj_set_size(bar5, LV_PCT(100), LV_PCT(100));
    lv_obj_set_grid_cell(bar5, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);

    lv_style_t label_style;
    lv_style_init(&label_style);
    lv_style_set_text_color(&label_style, lv_color_hex(0xffffff));
    lv_style_set_text_font(&label_style, &HarmonyOS_2bit);
    lv_obj_add_style(label0, &label_style, 0);
    lv_obj_add_style(label1, &label_style, 0);
    lv_obj_add_style(label2, &label_style, 0);
    lv_obj_add_style(label3, &label_style, 0);
    lv_obj_add_style(label4, &label_style, 0);
    lv_obj_add_style(label5, &label_style, 0);

    lv_style_t bar_style;
    lv_style_init(&bar_style);
    lv_style_set_bg_color(&bar_style, lv_color_hex(0x262626));
    lv_obj_add_style(bar0, &bar_style, 0);
    lv_obj_add_style(bar1, &bar_style, 0);
    lv_obj_add_style(bar2, &bar_style, 0);
    lv_obj_add_style(bar3, &bar_style, 0);
    lv_obj_add_style(bar4, &bar_style, 0);
    lv_obj_add_style(bar5, &bar_style, 0);

    config_lcd *lcd_data_temp;
    uint8_t ram_utilization_rate = 0, ram_used = 0, ram_total = 0, gpu_ram_utilization_rate = 0, gpu_ram_used = 0, gpu_ram_total = 0;
    while (1)
    {
        if (x_queue_lcd != 0 && xQueueReceive(x_queue_lcd, &(lcd_data_temp), 50))
        {
            switch (lcd_data_temp->command)
            {
            case LCD_COMMAND_CPU_TEMPERATUE:
                lv_label_set_text_fmt(label0, "CPU温度：%d℃", *lcd_data_temp->data);
                lv_bar_set_value(bar0, *lcd_data_temp->data, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar0, lv_color_hex(calc_color(*lcd_data_temp->data)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_CPU_UTILIZATION_RATE:
                lv_label_set_text_fmt(label1, "CPU利用率: %d%%", *lcd_data_temp->data);
                lv_bar_set_value(bar1, *lcd_data_temp->data, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar1, lv_color_hex(calc_color(*lcd_data_temp->data)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_RAM_UTILIZATION_RATE:
                ram_utilization_rate = *lcd_data_temp->data;
                lv_label_set_text_fmt(label2, "RAM利用率: %d%%（%dGB/%dGB）", ram_utilization_rate, ram_used, ram_total);
                lv_bar_set_value(bar2, ram_utilization_rate, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar2, lv_color_hex(calc_color(ram_utilization_rate)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_RAM_UESD:
                ram_used = *lcd_data_temp->data;
                lv_label_set_text_fmt(label2, "RAM利用率: %d%%（%dGB/%dGB）", ram_utilization_rate, ram_used, ram_total);
                break;
            case LCD_COMMAND_RAM_TOTAL:
                ram_total = *lcd_data_temp->data;
                lv_label_set_text_fmt(label2, "RAM利用率: %d%%（%dGB/%dGB）", ram_utilization_rate, ram_used, ram_total);
                break;
            case LCD_COMMAND_GPU_TEMPERATUE:
                lv_label_set_text_fmt(label3, "GPU温度：%d℃", *lcd_data_temp->data);
                lv_bar_set_value(bar3, *lcd_data_temp->data, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar3, lv_color_hex(calc_color(*lcd_data_temp->data)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_GPU_UTILIZATION_RATE:
                lv_label_set_text_fmt(label4, "GPU利用率: %d%%", *lcd_data_temp->data);
                lv_bar_set_value(bar4, *lcd_data_temp->data, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar4, lv_color_hex(calc_color(*lcd_data_temp->data)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_GPU_RAM_UTILIZATION_RATE:
                gpu_ram_utilization_rate = *lcd_data_temp->data;
                lv_label_set_text_fmt(label5, "GPU内存利用率: %d%%（%dGB/%dGB）", gpu_ram_utilization_rate, gpu_ram_used, gpu_ram_total);
                lv_bar_set_value(bar5, gpu_ram_utilization_rate, LV_ANIM_OFF);
                lv_obj_set_style_bg_color(bar5, lv_color_hex(calc_color(gpu_ram_utilization_rate)), LV_PART_INDICATOR);
                break;
            case LCD_COMMAND_GPU_RAM_UESD:
                gpu_ram_used = *lcd_data_temp->data;
                lv_label_set_text_fmt(label5, "GPU内存利用率: %d%%（%dGB/%dGB）", gpu_ram_utilization_rate, gpu_ram_used, gpu_ram_total);
                break;
            case LCD_COMMAND_GPU_RAM_TOTAL:
                gpu_ram_total = *lcd_data_temp->data;
                lv_label_set_text_fmt(label5, "GPU内存利用率: %d%%（%dGB/%dGB）", gpu_ram_utilization_rate, gpu_ram_used, gpu_ram_total);
            default:
                break;
            }
        }

        lv_timer_handler();
        vTaskDelay(1);
    }
}

void usb_uart_send(uint8_t *buf, uint8_t buf_len)
{
    for (uint8_t i = 0; i < buf_len; i++)
    {
        putchar_raw(buf[i]);
    }
}

void v_task_usb_uart(void *pvParameters)
{
    uint8_t buf[4], ret_buf[4];
    uint8_t buf_rx_i = 5, rx_pre = 0;
    buf[0] = 0x5a;
    while (1)
    {
        uint8_t dat;
        dat = getchar();
        // printf("%c", dat);

        if (dat == 0xa5 && rx_pre == 1)
        {
            buf_rx_i = 1;
        }

        if (dat == 0x5a)
        {
            rx_pre = 1;
        }
        else
        {
            rx_pre = 0;
        }

        if (buf_rx_i < 4)
        {
            buf[buf_rx_i] = dat;
            buf_rx_i++;
        }

        if (buf_rx_i == 4)
        {
            // printf("%s", buf);
            // 开始处理指令
            uint8_t cmd = buf[2];
            switch (cmd)
            {
            case 0xff:
                if (buf[3] == 0x01)
                {
                    ret_buf[0] = 0x5a;
                    ret_buf[1] = 0xa5;
                    ret_buf[2] = 0xff;
                    ret_buf[3] = 0x10;
                    usb_uart_send(ret_buf, 4);
                }
                break;
            case 0x01:
                x_queue_lcd_send(LCD_COMMAND_CPU_TEMPERATUE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x01;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x02:
                x_queue_lcd_send(LCD_COMMAND_CPU_UTILIZATION_RATE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x02;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x03:
                x_queue_lcd_send(LCD_COMMAND_RAM_UTILIZATION_RATE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x03;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x04:
                x_queue_lcd_send(LCD_COMMAND_RAM_UESD, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x04;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x05:
                x_queue_lcd_send(LCD_COMMAND_RAM_TOTAL, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x05;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x06:
                x_queue_lcd_send(LCD_COMMAND_GPU_TEMPERATUE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x06;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x07:
                x_queue_lcd_send(LCD_COMMAND_GPU_UTILIZATION_RATE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x07;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x08:
                x_queue_lcd_send(LCD_COMMAND_GPU_RAM_UTILIZATION_RATE, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x08;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x09:
                x_queue_lcd_send(LCD_COMMAND_GPU_RAM_UESD, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x09;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            case 0x0A:
                x_queue_lcd_send(LCD_COMMAND_GPU_RAM_TOTAL, &buf[3]);
                ret_buf[0] = 0x5a;
                ret_buf[1] = 0xa5;
                ret_buf[2] = 0x0A;
                ret_buf[3] = 0xff;
                usb_uart_send(ret_buf, 4);
                break;
            default:
                break;
            }
            buf_rx_i = 5;
        }
        vTaskDelay(1);
    }
}