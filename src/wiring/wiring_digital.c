/* *****************************************************************************
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Created: 12/18/09 02:42:00
 *  Copyright (c) 2009 Perry L. Hung. All rights reserved.
 *
 * ****************************************************************************/

/**
 *  @file wiring_digital.c
 *
 *  @brief 
 */

#include "wiring.h"
#include "io.h"
#include "gpio.h"

typedef enum AFMode{
    AF_NONE,
    AF_PWM,
    AF_SERIAL,
    AF_I2C,             // unused for now
    AF_SPI,             // unusued for now
} AFMode;


typedef struct PinGPIOMapping {
    GPIO_Port *port;
    uint32 pin;
} PinGPIOMapping;


/* Reset state is input floating  */
static const PinGPIOMapping PIN_TO_GPIO[NR_MAPLE_PINS] = {
    {GPIOA_BASE,  3},                          // D0/PA3
    {GPIOA_BASE,  2},                          // D1/PA2
    {GPIOA_BASE,  0},                          // D2/PA0
    {GPIOA_BASE,  1},                          // D3/PA1
    {GPIOB_BASE,  5},                          // D4/PB5
    {GPIOB_BASE,  6},                          // D5/PB6
    {GPIOA_BASE,  8},                          // D6/PA8
    {GPIOA_BASE,  9},                          // D7/PA9
    {GPIOA_BASE, 10},                          // D8/PA10
    {GPIOB_BASE,  7},                          // D9/PB7
    {GPIOA_BASE,  4},                          // D10/PA4
    {GPIOA_BASE,  7},                          // D11/PA7
    {GPIOA_BASE,  6},                          // D12/PA6
    {GPIOA_BASE,  5},                          // D13/PA5
/* for later  */
#if 0
    {GPIOC_BASE,  0},                          // D14/A0/PC0
    {GPIOC_BASE,  1},                          // D15/A1/PC1
    {GPIOC_BASE,  2},                          // D16/A2/PC2
    {GPIOC_BASE,  3},                          // D17/A3/PC3
    {GPIOC_BASE,  4},                          // D18/A4/PC4
    {GPIOC_BASE,  5},                          // D19/A5/PC5
#endif
};

void pinMode(uint8_t pin, WiringPinMode mode) {
    uint8 outputMode;

    if (pin >= NR_MAPLE_PINS)
        return;

    switch(mode) {
    case OUTPUT:
        outputMode = GPIO_MODE_OUTPUT_PP;
        break;
    case INPUT:
    case INPUT_FLOATING:
        outputMode = GPIO_MODE_INPUT_FLOATING;
        break;
    case INPUT_PULLUP:
        outputMode = GPIO_MODE_INPUT_PU;
        break;
    case INPUT_PULLDOWN:
        outputMode = GPIO_MODE_INPUT_PD;
        break;
    case PWM:
        outputMode = GPIO_MODE_AF_OUTPUT_PP;
        break;
    default:
        ASSERT(0);
        return;
    }


    gpio_set_mode(PIN_TO_GPIO[pin].port, PIN_TO_GPIO[pin].pin, outputMode);
}


uint32_t digitalRead(uint8_t pin) {
    if (pin >= NR_MAPLE_PINS)
        return 0;

    return (PIN_TO_GPIO[pin].port->IDR & BIT(PIN_TO_GPIO[pin].pin)) ? 1 : 0;
}

void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin >= NR_MAPLE_PINS)
        return;

    gpio_write_bit(PIN_TO_GPIO[pin].port, PIN_TO_GPIO[pin].pin, val);
}