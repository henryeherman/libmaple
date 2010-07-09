
#include "libmaple.h"
#include "libmaple_types.h"
#include "rcc.h"
#include "nvic.h"
#include "wiring.h"
#include "wiring_math.h"
#include "HardwareSerial.h"
#include "usb.h"
#include "usart.h"
#include "i2c.h"


#define I2C_SLAVE  1
#define I2C_MASTER 2

//HardwareUsb usb;

int ledPin = 13;
int toggle = 1;
uint8 data[7]; 

void setup() {
    pinMode(ledPin, OUTPUT);
    data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = 100; 
    i2c_init(I2C_PORT1, 10000);
    Serial2.begin(9600);

}

void loop() {
    uint8 n = 0; n++;// TODO
    digitalWrite(ledPin, 1);
    delay(500);
    digitalWrite(ledPin, 0);
    delay(500);
    
    //toggle ^= 1;
    //digitalWrite(ledPin, toggle);

    // reset i2c
    //i2c_init(I2C_PORT1, 10000);
    // go
    //i2c_send1(4,5);
    //i2c_slave_set_addr(I2C_PORT1, 4);
    //i2c_send1(4,5);
    //n = i2c_isbusy(I2C_PORT1); Serial2.print("pre1busy = "); Serial2.println((uint32)n);
    data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = 64; 
    i2c_master_write(I2C_PORT1, 4, data, 1);
    //n = i2c_read1(4);
    
    /*
    data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = 100; 
    i2c_master_read(I2C_PORT1, 4, data, 6);
    delay(100);
    i2c_isbusy(I2C_PORT1);
    Serial2.println("----------");

    Serial2.print("data[0] = "); Serial2.println((uint32)data[0],DEC);
    Serial2.print("data[1] = "); Serial2.println((uint32)data[1],DEC);
    Serial2.print("data[2] = "); Serial2.println((uint32)data[2],DEC);
    Serial2.print("data[3] = "); Serial2.println((uint32)data[3],DEC);
    Serial2.print("data[4] = "); Serial2.println((uint32)data[4],DEC);
    Serial2.print("data[5] = "); Serial2.println((uint32)data[5],DEC);
    //Serial2.print("postread = "); Serial2.println((uint32)i2c_getlen(I2C_PORT1));
    */ 
    //n = i2c_isbusy(I2C_PORT1); Serial2.print("post2busy = "); Serial2.println((uint32)n);
    //Serial2.print("len = "); Serial2.println(i2c_getlen(I2C_PORT1));

    //i2c_start();
    /*
    Serial2.print("Value: b");
    Serial2.print(1 && (n & 0x80));
    Serial2.print(1 && (n & 0x40));
    Serial2.print(1 && (n & 0x20));
    Serial2.print(1 && (n & 0x10));
    Serial2.print(1 && (n & 0x08));
    Serial2.print(1 && (n & 0x04));
    Serial2.print(1 && (n & 0x02));
    Serial2.println(1 && (n & 0x01));
    */
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
