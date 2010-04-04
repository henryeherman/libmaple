/* *****************************************************************************
 * The MIT License
 *
 *  Created: Thu Mar 18 13:54:17
 *  Copyright (c) 2010 Bryan Newbold
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
 * ****************************************************************************/

// See also ../../notes/i2c

/**
 *  @file i2c.c
 *
 *  @brief i2c control routines
 */

#include "libmaple.h"
#include "i2c.h"
#include "gpio.h"
#include "nvic.h"

#define I2C_RECV_BUF_SIZE 128
#define I2C_MAX_FREQ 400000

#define IS_VALID_I2C_NUM(NUM) (NUM == 1) || (NUM == 2)

#define I2C_SLAVE  1
#define I2C_MASTER 2

/* Ring buffer notes:
 * Copied from perry's USART ring buffer
 * The buffer is empty when head == tail.
 * The buffer is full when the head is one byte in front of the tail
 * The total buffer size must be a power of two
 * Note, one byte is necessarily left free with this scheme */
typedef struct i2c_ring_buf {
    uint32 head;
    uint32 tail;
    uint8 buf[I2C_RECV_BUF_SIZE];
} i2c_ring_buf;

// Follow pin naming convention (1,2 not 0,1) for i2c ports
static i2c_ring_buf ring_buf1;
static i2c_ring_buf ring_buf2;

/*
void I2C1_IRQHandler(void) {
    //  Read the data  
    ring_buf1.buf[ring_buf1.tail++] = (uint8_t)(((i2c_port*)(I2C1_BASE))->DR);
    ring_buf1.tail %= I2C_RECV_BUF_SIZE;
}
*/

/**
 *  @brief Readies the port and sets frequency
 *
 */
void i2c_init(uint8 i2c_num, uint16 freq) {
    uint16 freqrange=0, tmp=0;
    uint32 pclk1=8000000;
    i2c_port *port;
    i2c_ring_buf *ring_buf;
    RCC_ClocksTypeDef rcc_clocks;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));
    ASSERT(freq && (freq <= I2C_MAX_FREQ)); // TODO

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        ring_buf = &ring_buf1;
        /* Enable I2C1 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        /* Enable GPIOB clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        /* Configure I2C1 pins: SCL and SDA -------------------------------*/
        gpio_set_mode(GPIOB_BASE, 6, GPIO_MODE_AF_OUTPUT_OD);
        gpio_set_mode(GPIOB_BASE, 7, GPIO_MODE_AF_OUTPUT_OD);
        /* Configure and enable I2C1 event and error interrupts -----------*/
        nvic_enable_interrupt(NVIC_INT_I2C1_EV);
        nvic_enable_interrupt(NVIC_INT_I2C1_ER);
        /*
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        */
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        ring_buf = &ring_buf2;
        /* Enable I2C2 clock */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        /* Enable GPIOB clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        /* Configure I2C2 pins: SCL and SDA -------------------------------*/
        gpio_set_mode(GPIOB_BASE, 10, GPIO_MODE_AF_OUTPUT_OD);
        gpio_set_mode(GPIOB_BASE, 11, GPIO_MODE_AF_OUTPUT_OD);
        /* Configure and enable I2C2 event and error interrupts -----------*/
        nvic_enable_interrupt(NVIC_INT_I2C2_EV);
        nvic_enable_interrupt(NVIC_INT_I2C2_ER);
        /*
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_Init(&NVIC_InitStructure);
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
        NVIC_Init(&NVIC_InitStructure);
        */
        break;
    default:
        /* should never get here  */
        ASSERT(0);
    }

    /* Initialize ring buffer  */
    ring_buf->head = 0;
    ring_buf->tail = 0;


    // Set clock based on PCLK1 frequency value
    RCC_GetClocksFreq(&rcc_clocks);
    pclk1 = rcc_clocks.PCLK1_Frequency;
    freqrange = (uint16)(pclk1 / 1000000);
    port->CR2 |= (port->CR2 & 0xFFC0) | freqrange;

    // Set port speed: either fast mode or slow mode
    port->CR1 &= (~ I2C_CR1_PE); // Disable port to configure TRISE

    if (freq <= 100000) {   // Configure speed in standard mode 
        tmp = (uint16)(pclk1 / (freq << 1));
        if (tmp < 0x04) {   // Set minimum allowed value
            tmp = 0x04;  
        }
        /* Set speed value for standard mode */
        port->CCR |= tmp;
        /* Set Maximum Rise Time for standard mode */
        port->TRISE = freqrange + 1;
     } else {               // Configure speed in fast mode
        if (1) {            // Use Tlow/Thigh = 2 duty cycleby default
            /* Fast mode speed calculate: Tlow/Thigh = 2 */
            tmp = (u16)(pclk1 / (freq * 3));
        } else {
            /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
            tmp = (u16)(pclk1 / (freq * 25));
            /* Set DUTY bit */
            tmp |= I2C_CCR_DUTY;
        }
        if ((tmp & 0x0FFF) == 0) { /* Set minimum allowed value */
            tmp |= (u16)0x0001;
        }
        /* Set speed value and set F/S bit for fast mode */
        port->CCR |= (tmp & (uint16)0x003F) | I2C_CCR_FS;
        /* Set Maximum Rise Time for fast mode */
        port->TRISE = (uint16)(((freqrange * 300) / 1000) + 1);
    }
    port->CR1 |= I2C_CR1_PE; // Re-enable port after configuring TRISE

    // Set CR1 defaults: 
    //  disabled: PEC, SMBUS, SMBUSTYPE
    //  enabled: START, STOP, NOSTRETCH, ACK
    // TODO: is this the correct clear syntax?
    port->CR1 &= (~ (I2C_CR1_PEC | I2C_CR1_SMBUS | I2C_CR1_SMBTYPE)); 
    port->CR1 |= I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_NOSTRETCH | I2C_CR1_ACK;

    // Set OAR1 defaults: 
    //  disabled: ADDMODE
    port->OAR1 = 0x0000 | I2C_DEFAULT_SLAVE_ADDRESS;

    // Set OAR2 defaults: 
    //  disabled: ENDUAL
    port->OAR1 = 0x0000;

    // Enable port event and buffer interrupts
    port->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN;

    // Enable port
    port->CR1 |= I2C_CR1_PE;
}


/**
 *  @brief Turns off an i2c port
 *
 */
void i2c_disable(uint8 i2c_num) {
    i2c_port *port;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }

    // Unset enable pin
    port->CR1 &= (~ I2C_CR1_PE);
    // TODO: anything else here?
}

/**
 *  @brief Sets port mode (I2C_SLAVE or I2C_MASTER)
 *
 */
void i2c_set_mode(uint8 i2c_num, uint8 mode) {
    i2c_port *port;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));
    ASSERT((mode == I2C_SLAVE) || (mode == I2C_MASTER));

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }

    // TODO: not sure what to do here
}

/**
 *  @brief Become master and read data from a given slave device
 *
 */
uint8 i2c_master_read(uint8 i2c_num, uint8 addr, uint8 *data, uint8 length) {
    i2c_port *port;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }
    
    i2c_set_mode(i2c_num, I2C_MASTER); // don't always have to do this...

    // TODO: finish
    return '\0';
}


/**
 *  @brief Becomes master and writes data to a given slave device
 *
 */
uint8 i2c_master_write(uint8 i2c_num, uint8 addr, uint8* data, uint8 length, uint8 wait) {
    i2c_port *port;
    i2c_ring_buf *ring_buf;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        ring_buf = &ring_buf1;
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        ring_buf = &ring_buf2;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }
    
    i2c_set_mode(i2c_num, I2C_MASTER); // don't always have to do this...

    // Check that there is something in the buffer
    // TODO: it

    /* Send I2C1 START condition */
    port->CR1 |= I2C_CR1_START;

    /* Wait until all data and the PEC value are received */
    /* I2C2_Buffer_Rx buffer will contain the data plus the PEC value */
    while(ring_buf->head != ring_buf->tail) { // TODO
    }

    return '\0';
}

/**
 *  @brief Sets slave address (?)
 *
 */
void i2c_slave_set_addr(uint8 i2c_num, uint8 addr) {
    i2c_port *port;

    // Check parameters
    ASSERT(IS_VALID_I2C_NUM(i2c_num));

    switch (i2c_num) {
    case 1:
        port = (i2c_port*)I2C1_BASE;
        break;
    case 2:
        port = (i2c_port*)I2C2_BASE;
        break;
    default:
        ASSERT(0); // shouldn't ever get here
    }

    i2c_set_mode(i2c_num, I2C_SLAVE); // don't always have to do this...
    port->OAR1 = (uint16)(0x0000 | (0x007F & (uint16)addr));
        
}


/**
 *  @brief Sets the RX callback (for operating in slave mode)
 *
 */
void i2c_slave_set_rx_callback(uint8 i2c_num, void (*function)(uint8*, int)) {
    //TODO
    ASSERT(IS_VALID_I2C_NUM(i2c_num));
    return;
}

/**
 *  @brief Sets the TX callback (for operating in slave mode)
 *
 */
void i2c_slave_set_tx_callback(uint8 i2c_num, void (*function)(void)) {
    // TODO
    ASSERT(IS_VALID_I2C_NUM(i2c_num));
    return;
}