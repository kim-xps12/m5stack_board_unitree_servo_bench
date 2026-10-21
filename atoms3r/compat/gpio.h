// Stand-in for the STM32 HAL "gpio.h" that stm32/firmware/App/protocol.h includes.
// protocol.h uses nothing from it; crc_ccitt.h only needs size_t.
#pragma once
#include <stddef.h>
