#include "libmaple.h"
#include "libmaple_types.h"
#include "wiring.h"
#include "HardwareSerial.h"
#include "wiring_math.h"
#include "HardwareUsb.h"
#include "usb.h"
#include "i2c.h"
#include "usart.h"

int ledPin = 13;
//HardwareUsb Usb;

void setup() {
    pinMode(ledPin, OUTPUT);
    i2c_init(1,100000);
}

int toggle = 1;
void loop() {
    digitalWrite(ledPin, toggle);
    toggle ^= 1;
    delay(1000);
    //Usb.println("hello world!");
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
