#pragma once
constexpr unsigned GPIO_OVERRIDE_NORMAL = 0;
constexpr unsigned GPIO_OVERRIDE_LOW = 2;
bool gpio_get(unsigned pin);
void gpio_set_outover(unsigned pin, unsigned value);
