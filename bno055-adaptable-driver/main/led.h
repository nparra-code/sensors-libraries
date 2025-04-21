#ifndef LED_H
#define LED_H

#include <led_strip.h>
#include <driver/gpio.h>
#include <esp_log.h>

#define BLINK_GPIO 48

typedef struct {
    led_strip_handle_t led_strip;
    uint8_t led_state;
} led_t;

void configure_led(led_t *led) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led->led_strip));
    led->led_state = 0;
    led_strip_clear(led->led_strip);
}

void blink_led(led_t *led) {
    if (led->led_state) {
        led_strip_set_pixel(led->led_strip, 0, 16, 16, 16);
        led_strip_refresh(led->led_strip);
    } else {
        led_strip_clear(led->led_strip);
    }
}

void led_green(led_t *led, uint8_t value) {
    led_strip_set_pixel(led->led_strip, 0, 0, value, 0);
    led_strip_refresh(led->led_strip);
}

void led_red(led_t *led, uint8_t value) {
    led_strip_set_pixel(led->led_strip, 0, value, 0, 0);
    led_strip_refresh(led->led_strip);
}

void led_blue(led_t *led, uint8_t value) {
    led_strip_set_pixel(led->led_strip, 0, 0, 0, value);
    led_strip_refresh(led->led_strip);
}

void led_set_color(led_t *led, uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led->led_strip, 0, r, g, b);
    led_strip_refresh(led->led_strip);
}

void led_clear(led_t *led) {
    led_strip_clear(led->led_strip);
}


#endif // LED_H