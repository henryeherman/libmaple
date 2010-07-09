/* *****************************************************************************
 * The MIT License
 *
 *  Created: Thu Mar 18 13:45:09
 *  Copyright (c) 2010 Bryan Newbold. All rights reserved.
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

/**
 *  @brief 
 */

#ifndef _I2C_H_
#define _I2C_H_

#define I2C1_BASE       0x40005400
#define I2C2_BASE       0x40005800

#ifdef __cplusplus
extern "C"{
#endif

#define I2C_TIMEOUT     100 // in milliseconds

#define I2C_PORT1   1
#define I2C_PORT2   2

// Slave address for Leaflabs Maple Board. TODO: choose a reasonable value?
#define I2C_DEFAULT_SLAVE_ADDRESS ((uint16)42)

// I2C acknowledged address defines
#define I2C_ACK_ADDRESS_7BIT ((uint16)0x4000)
#define I2C_ACK_ADDRESS_10BIT ((uint16)0xC000)

// ((uint32)0x40000000) = PERIPH_BASE = APB1PERIPH_BASE
#define I2C1_BASE             (((uint32)0x40000000) + 0x5400)
#define I2C2_BASE             (((uint32)0x40000000) + 0x5800)

// Full status flag definitions ---------------------------------------------
// CR2 register flags
#define I2C_CR2_LAST       ((uint16)0x0800) // DMA last transfer
#define I2C_CR2_DMAEN      ((uint16)0x0400) // DMA requests enable
#define I2C_CR2_ITBUFEN    ((uint16)0x0200) // Buffer interrupt enable
#define I2C_CR2_ITEVTEN    ((uint16)0x0100) // Event interrupt enable
#define I2C_CR2_ITERREN    ((uint16)0x0080) // Error interrupt enable
// values as a mask?
//#define I2C_CR2_FREQ       ((uint16)0x003F) // Frequency (6bit, 2-36 allowed,
                                            // Mhz)
// CR1 register flags
#define I2C_CR1_SWRST      ((uint16)0x8000) // Software reset
#define I2C_CR1_ALERT      ((uint16)0x2000) // SMBus alert
#define I2C_CR1_PEC        ((uint16)0x1000) // Packet error checking
#define I2C_CR1_POS        ((uint16)0x0800) // Acknowledge/PEC position (recvr)
#define I2C_CR1_ACK        ((uint16)0x0400) // Acknowledge enable
#define I2C_CR1_STOP       ((uint16)0x0200) // Stop generation
#define I2C_CR1_START      ((uint16)0x0100) // Start generation
#define I2C_CR1_NOSTRETCH  ((uint16)0x0080) // Clock stretching disable (slave)
#define I2C_CR1_ENGC       ((uint16)0x0040) // General call enable
#define I2C_CR1_ENPEC      ((uint16)0x0020) // PEC enable
#define I2C_CR1_ENARP      ((uint16)0x0010) // ARP enable (smbus)
#define I2C_CR1_SMBTYPE    ((uint16)0x0008) // SMBus type
#define I2C_CR1_SMBUS      ((uint16)0x0002) // SMBus mode
#define I2C_CR1_PE         ((uint16)0x0001) // Peripheral enable
// OAR1 register flags 
#define I2C_OAR1_ADDMODE   ((uint16)0x8000) // Addressing mode (slave, 0=7bit)
// OAR2 register flags 
#define I2C_OAR2_ENDUAL    ((uint16)0x0001) // Dual addressing mode enable
// SR1 register flags
#define I2C_SR1_SMBALERT   ((uint16)0x8000) // SMBus alert
#define I2C_SR1_TIMEOUT    ((uint16)0x4000) // Timeout or Tlow error
#define I2C_SR1_PECERR     ((uint16)0x1000) // PEC Error in reception
#define I2C_SR1_OVR        ((uint16)0x0800) // Overrun/Underrun
#define I2C_SR1_AF         ((uint16)0x0400) // Acknowledge failure
#define I2C_SR1_ARLO       ((uint16)0x0200) // Arbitration lost (master)
#define I2C_SR1_BERR       ((uint16)0x0100) // Bus error
#define I2C_SR1_TXE        ((uint16)0x0080) // Data register empty (trans)
#define I2C_SR1_RXNE       ((uint16)0x0040) // Data register not empty (rcv)
#define I2C_SR1_STOPF      ((uint16)0x0010) // Stop detection (slave)
#define I2C_SR1_ADD10      ((uint16)0x0008) // 10bit header sent (master)
#define I2C_SR1_BTF        ((uint16)0x0004) // Byte transfer finished
#define I2C_SR1_ADDR       ((uint16)0x0002) // Address sent (master)
                                            // or address matched (slave)
#define I2C_SR1_SB        ((uint16)0x0001) // Start bit (master)
// SR2 register flags 
#define I2C_SR2_DUALF      ((uint16)0x0080) // Dual flag (slave)
#define I2C_SR2_SMBHOST    ((uint16)0x0040) // SMBus host header (slave)
#define I2C_SR2_SMBDEFAULT ((uint16)0x0020) // SMBus device default address
                                            // (slave)
#define I2C_SR2_GENCALL    ((uint16)0x0010) // General call address (slave)
#define I2C_SR2_TRA        ((uint16)0x0004) // Transmitter/receiver
#define I2C_SR2_BUSY       ((uint16)0x0002) // Bus busy
#define I2C_SR2_MSL        ((uint16)0x0001) // Master/Slave (1=master)
// CCR register flags 
#define I2C_CCR_FS         ((uint16)0x8000) // Master mode speed (1=fast)
#define I2C_CCR_DUTY       ((uint16)0x4000) // Fast mode duty cycle (1=16/9)

// Mapping of I2C1 or I2C2 registers into a struct --------------------------
typedef struct i2c_port { 
    volatile uint16 CR1;      // Control register 1
    uint16 RESERVED0;
    volatile uint16 CR2;      // Control register 2
    uint16 RESERVED1;
    volatile uint16 OAR1;     // Own Address register 1
    uint16 RESERVED2;
    volatile uint16 OAR2;     // Own Address register 2
    uint16 RESERVED3;
    volatile uint16 DR;       // Data register
    uint16 RESERVED4;
    volatile uint16 SR1;      // Status register
    uint16 RESERVED5;
    volatile uint16 SR2;      // Status register
    uint16 RESERVED6;
    volatile uint16 CCR;      // Clock Control register
    uint16 RESERVED7;
    volatile uint16 TRISE;    // TRISE register
} i2c_port;


// Function definitions -----------------------------------------------------
// FOR TESTING
void i2c_send1(uint32 addr, uint32 data);
uint8 i2c_read1(uint32 addr);
uint32 i2c_getlen(uint8 i2c_num);
uint8 i2c_isbusy(uint8 i2c_num);

void i2c_init(uint32 i2c_num, uint32 freq);
void i2c_disable(uint32 i2c_num);

void i2c_master_read(uint8 i2c_num, uint32 addr, uint8 *data, uint32 len);
void i2c_master_write(uint8 i2c_num, uint32 addr, uint8 *data, uint32 len);

void i2c_slave_set_addr(uint32 port, uint32 addr);
void i2c_slave_set_begin_callback(uint32 port, void (*function)(void));
void i2c_slave_set_rx_callback(uint32 port, void (*function)(uint8*));
void i2c_slave_set_tx_callback(uint32 port, uint8 (*function)(void));
void i2c_slave_set_end_callback(uint32 port, void (*function)(void));

void default_slave_callback(void);
void default_rx_callback(uint8 data);
uint8 default_tx_callback(void);

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif

