
#include "libmaple.h"
#include "libmaple_types.h"
#include "rcc.h"
#include "nvic.h"
#include "wiring.h"
#include "wiring_math.h"
#include "HardwareUsb.h"
#include "usb.h"
#include "usart.h"
#include "i2c.h"


#define I2C_SLAVE  1
#define I2C_MASTER 2
i2c_port *port;
uint8 color = 4;
uint8 byte_count = 0;


int ledPin = 13;
int toggle = 1;
//HardwareUsb Usb;

void setup() {
    pinMode(ledPin, OUTPUT);

    port = (i2c_port*)I2C1_BASE;

    port->CR1 |= I2C_CR1_SWRST;  // reset the peripheral
    port->CR1 &= ~I2C_CR1_SWRST;  
    
    // disable the hardware to configure
    port->CR1 &= (~ I2C_CR1_PE); 

    // - setup pins
    /* Configure I2C1 pins: SCL and SDA */
    // on the maple, I2C1 SCL is Pin 5 and I2C1 SDA is Pin 9
    gpio_set_mode(GPIOB_BASE, 6, GPIO_MODE_AF_OUTPUT_OD); // NOT maple pin #s
    gpio_set_mode(GPIOB_BASE, 7, GPIO_MODE_AF_OUTPUT_OD);

    // - setup peripheral input clock: 4MHz for fast mode (TODO?)
    rcc_enable_clk_gpiob(); /* Enable GPIOB clock */
    rcc_enable_clk_i2c1(); /* Enable I2C1 clock */
    port->CR2 |= (port->CR2 & 0xFFC0) | 8; // set clock to 8MHz

    // - configure interupts
    nvic_enable_interrupt(NVIC_INT_I2C1_EV);
    nvic_enable_interrupt(NVIC_INT_I2C1_ER);

    // - configure clock control registers
    /* Set speed value for standard mode */
    port->CCR = 0x28; // 100kHz
    port->CCR &= (~ I2C_CCR_FS); // standard mode
    
    // - configure rise time register
    /* Set Maximum Rise Time for standard mode */
    port->TRISE = 9; // for 100kHz standard

    // - configure I2C_CR1, I2C_CR2 to enable the peripheral
    port->CR1 &= (~ (I2C_CR1_PEC | I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_NOSTRETCH )); 
    port->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

    // - set ACK flag low: won't be a slave until addr is set
    port->CR1 &= (~ I2C_CR1_ACK);
    
    // Re-enable port after configuration
    port->CR1 |= I2C_CR1_PE; 
}

void loop() {
    // - software sets START high, which toggles M/SL to master mode
    port->CR1 |= I2C_CR1_START; 
    
    //Usb.println("hello world!");
    digitalWrite(ledPin, toggle);
    toggle ^= 1;
    delay(1000);
}

void I2C1_EV_IRQHandler(void) {
    uint16 SR1 = port->SR1;
    uint16 SR2 = port->SR2;
    
    if(SR1 & I2C_SR1_SB) {
        // - when START actually happens, SB is set and interrupt happens; hardware
        //   waits until address is written to DR
        port->DR = 45; // ADDR of slave device (led thing #45)
        byte_count = 1;
    } else if((SR1 & I2C_SR1_ADDR) && (SR2 & I2C_SR2_TRA)) {
        // - address shifts out and an interrupt is thrown with ADDR high; if LSB of
        //   address was low, in transmitter mode. TRA reflects this
        // - software writes to the first byte to DR and clears ADDR
        port->DR = color;
        port->SR1 &= ~ I2C_SR1_ADDR;
        byte_count--;
    } else if((SR1 & I2C_SR1_TXE) && (SR1 & I2C_SR1_BTF)) {
        // - first byte shifts out and when there's an ACK an interrupt is thrown
        //   with TxE high; if no new byte was written to DR since the previous
        //   transmission BTF goes high
        // - software writes next byte to DR and clears BTF, or sets STOP bit to end
        //   data transmission, or sets START to begin next part of combined session
        // - after STOP is set, hardware goes back to slave mode
        if(byte_count) {
            port->DR = color;
            port->SR1 &= ~I2C_SR1_BTF;
            byte_count--;
        } else {
            port->DR = color;
            port->SR1 &= ~I2C_SR1_BTF;
            port->SR1 |= I2C_SR1_STOPF;
        }

    }
}

int main(void) {
    init();
    setup();

    while (1) {
        loop();
    }
    return 0;
}

/* Required for C++ hackery */
/* TODO: This really shouldn't go here... move it later
 * */
extern "C" void __cxa_pure_virtual(void) {
    while(1)
        ;
}
