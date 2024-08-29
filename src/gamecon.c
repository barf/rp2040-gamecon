/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// https://forums.raspberrypi.com/viewtopic.php?t=337719#p2025214
#define NUM_FRAC    16
#define FLOAT_TO_FIX_SCALE  65536.0 // 1<<16
#define MULT_SHIFT  16
#define FIX_TO_FLOAT    (1.0/65536.0)

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>

#include "bsp/board.h"
#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/printf.h"
#include "pico/stdio/driver.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

void hid_task(void);
char debug_buffer[80];
uint16_t sma_buffer[1024];

struct report
{
    uint16_t buttons;
    uint8_t joy0;
    uint8_t joy1;
    uint8_t joy2;
    uint8_t joy3;
} report;

struct filter
{
    uint16_t floor;
    uint16_t ceiling;
} filter;

int main(void)
{
    filter.floor = 65535;
    filter.ceiling = 0;

    board_init();
    tusb_init();

    adc_init();
    adc_set_temp_sensor_enabled(false);
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    // adc_gpio_init(29);

    uart_init(uart0, 115200);
    gpio_set_function(0, UART_FUNCSEL_NUM(uart0, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(uart0, 1));

    stdio_init_all();
    uart_puts(uart0, "INIT OK\r\n");

#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

    while (1)
    {
        hid_task();
        tud_task(); // tinyusb device task
    }

    return 0;
}

void con_panic(uint16_t errcode)
{
    report.buttons = errcode;
    while (1)
    {
        tud_task(); // tinyusb device task
        // Remote wakeup
        if (tud_suspended())
        {
            // Wake up host if we are in suspend mode
            // and REMOTE_WAKEUP feature is enabled by host
            tud_remote_wakeup();
        }

        if (tud_hid_ready())
        {
            tud_hid_n_report(0x00, 0x01, &report, sizeof(report));
        }
    }
}

typedef struct
{
    uint8_t r, g, b;
} RGB_t;

union
{
    struct
    {
        uint8_t buttons[16];
        RGB_t rgb[3];
    } lights;
    uint8_t raw[25];
} light_data;

/* Choose 'C' for Celsius or 'F' for Fahrenheit. */
#define TEMPERATURE_UNITS 'C'

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
float read_onboard_temperature(const char unit) {
    
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

// https://stackoverflow.com/a/29135850
uint16_t roundup(uint16_t m, uint16_t n)
{
 return (m + n - 1) / n  ;
}

uint16_t rounddown(uint16_t m, uint16_t n)
{
 return m / n;
}

uint16_t round_uint16(uint16_t m, uint16_t n)
{
   int mod = m % n;

   if(mod >= (n + 1) / 2)
      return roundup(m, n);
   else
      return rounddown(m, n);
}

// gamecon.c
void hid_task(void)
{
    const uint16_t h_floor = 1024;
    const uint16_t h_ceiling = 2048;

    // Poll every 1ms
    const uint32_t interval_ms = 1;
    static uint32_t start_ms = 0;

    // adc conversion factor: 3.3v ref
    const float conversion_factor = 3.3f / (1 << 12);

    if (board_millis() - start_ms < interval_ms)
        return; // not enough time
    start_ms += interval_ms;

    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    uint16_t result_ADC0 = adc_read();
    adc_select_input(1);
    uint16_t result_ADC1 = adc_read();
    adc_select_input(2);
    uint16_t result_ADC2 = adc_read();

    // auto scaling?
    if (result_ADC2 < filter.floor) {
        // 0x431 // 1073
        filter.floor = result_ADC2;
    }
    if (result_ADC2 > filter.ceiling) {
        // 0x7c1 // 1985
        filter.ceiling = result_ADC2;
    }

    // clamping
    uint16_t axis_0;
    if (result_ADC2 < h_floor) {
        result_ADC2 = h_floor;
    } else if (result_ADC2 > h_ceiling) {
        result_ADC2 = h_ceiling;
    } else {
        // scale and range to uint8
        // uint16_t axis_0 = (result_ADC2 - filter.floor) / (filter.ceiling/256); // truncates
        axis_0 = round_uint16((result_ADC2 - h_floor), (h_ceiling/256));
    }

    report.buttons = 0;
    report.joy0 = (uint8_t)axis_0;
    report.joy1 = (uint8_t)filter.floor;
    report.joy2 = (uint8_t)filter.ceiling;
    report.joy3 = 0;

    // Remote wakeup
    if (tud_suspended())
    {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    }

    if (tud_hid_ready())
    {
        tud_hid_n_report(0x00, 0x01, &report, sizeof(report));
    }

    // debug strings on UART0
    // snprintf(debug_buffer, sizeof debug_buffer, "ADC0:0x%03x\tADC1:0x%03x\tADC2:0x%03x\r\n", result_ADC0, result_ADC1, result_ADC2);
    // uart_puts(uart0, debug_buffer);
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    if (report_id == 2 && report_type == HID_REPORT_TYPE_OUTPUT && buffer[0] == 2 && bufsize >= sizeof(light_data)) //light data
    {
        size_t i = 0;
        for (i; i < sizeof(light_data); i++)
        {
            light_data.raw[i] = buffer[i + 1];
        }
    }

    // echo back anything we received from host
    tud_hid_report(0, buffer, bufsize);
}
