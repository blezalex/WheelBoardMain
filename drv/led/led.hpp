#pragma once

#include <cinttypes>

#define LED_COUNT 20

void led_set_color(int led_idx, uint32_t color);
void led_init();
