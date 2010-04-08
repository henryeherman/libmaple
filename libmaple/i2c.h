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

#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 64
#endif

#define I2C_READY 0
#define I2C_MRX   1
#define I2C_MTX   2
#define I2C_SRX   3
#define I2C_STX   4

// Slave address for Leaflabs Maple Board. TODO: choose a reasonable value?
#define I2C_DEFAULT_SLAVE_ADDRESS ((uint16)42)

// I2C acknowledged address defines
#define I2C_ACK_ADDRESS_7BIT ((uint16)0x4000)
#define I2C_ACK_ADDRESS_10BIT ((uint16)0xC000)

// ((uint32)0x40000000) = PERIPH_BASE = APB1PERIPH_BASE
#define I2C1_BASE             (((uint32)0x40000000) + 0x5400)
#define I2C2_BASE             (((uint32)0x40000000) + 0x5800)

// I2C interrupt class definition --------------------------------------------
#define I2C_IT_BUF                      ((uint16)0x0400) // TODO: or 0x0600?
#define I2C_IT_EVT                      ((uint16)0x0200)
#define I2C_IT_ERR                      ((uint16)0x0100)

// Full interrupt definitions (see pg652/1003) -------------------------------
#define I2C_IT_SMBALERT ((uint32)0x01008000) // SMBus Alert
#define I2C_IT_TIMEOUT  ((uint32)0x01004000) // Timeout/TLow error
#define I2C_IT_PECERR   ((uint32)0x01001000) // PEC error
#define I2C_IT_OVR      ((uint32)0x01000800) // Overrun/Underrun
#define I2C_IT_AF       ((uint32)0x01000400) // Acknowledge failure
#define I2C_IT_ARLO     ((uint32)0x01000200) // Arbitration loss (master)
#define I2C_IT_BERR     ((uint32)0x01000100) // Bus error
#define I2C_IT_TXE      ((uint32)0x06000080) // Transmit buffer empty
#define I2C_IT_RXNE     ((uint32)0x06000040) // Receive buffer not empty
#define I2C_IT_STOPF    ((uint32)0x02000010) // Stop received (slave)
#define I2C_IT_ADD10    ((uint32)0x02000008) // 10bit header sent (master)
#define I2C_IT_BTF      ((uint32)0x02000004) // Data byte transfer finished
#define I2C_IT_ADDR     ((uint32)0x02000002) // Address sent (master)
                                          // or Address matched (slave)
#define I2C_IT_SB       ((uint32)0x02000001) // Start bit sent (master)

// Full status flag definitions ---------------------------------------------
// CR2 register flags
#define I2C_CR2_LAST       ((uint16)0x0800) // DMA last transfer
#define I2C_CR2_DMAEN      ((uint16)0x0400) // DMA requests enable
#define I2C_CR2_ITBUFEN    ((uint16)0x0200) // Buffer interrupt enable
#define I2C_CR2_ITEVTEN    ((uint16)0x0100) // Event interrupt enable
#define I2C_CR2_ITERREN    ((uint16)0x0080) // Error interrupt enable
// TODO: values as a mask?
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

// Mapping of I2C1 or I2C2 registers into a struct
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
void i2c_init(uint8 i2c_num, uint16 freq);
void i2c_set_mode(uint8 i2c_num, uint8 mode);
void i2c_disable(uint8 i2c_num);

// these will go to master mode if not already
uint8 i2c_master_read(uint8 port, uint8 addr, uint8 *data, uint8 length);
uint8 i2c_master_write(uint8 port, uint8 addr, uint8* data, 
                       uint8 length, uint8 wait);    // need length?

// these will go to slave mode if not already
void i2c_slave_set_addr(uint8 port, uint8 addr);
void i2c_slave_set_rx_callback(uint8 port, void (*function)(uint8*, int));
void i2c_slave_set_tx_callback(uint8 port, void (*function)(void));


/*
void i2c_setAddress(uint8 i2c_num, uint8 address);
uint8 i2c_readFrom(uint8 i2c_num, uint8 address, uint8* data, uint8 length);
uint8 i2c_writeTo(uint8 i2c_num, uint8 address, uint8* data, uint8 length, uint8 wait);
uint8 i2c_transmit(uint8 i2c_num, uint8* data, uint8 length);
// TODO: what should the int parameter actually be?
void i2c_attachSlaveRxEvent(uint8 i2c_num, void (*)(uint8*, int) );
void i2c_attachSlaveTxEvent(uint8 i2c_num, void (*)(void) );
void i2c_reply(unit8 i2c_num, uint8 ack);
void i2c_stop(unit8 i2c_num);
void i2c_releaseBus(unit8 i2c_num);
*/

#ifdef __cplusplus
} // extern "C"
#endif

#endif

