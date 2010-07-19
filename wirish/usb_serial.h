/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
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
 *  @brief wirish usb class for easy goin communication, uses libmaple's
 *  virtual com port implementation
 */

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

#define USB_DISABLED   0
#define NONBLOCKING    1
#define BLOCKING       2
#define TIMEOUT        3

class USBSerial {

    private:
        uint32 timeout;
        uint8  mode;

        // Print.h functions with return values
        uint32 printNumber(unsigned long, uint8);
        uint32 printFloat(double, uint8);

    public:
        USBSerial(void);

        void begin(void);
        void begin(uint8);
        void begin(uint8, uint32);
        void end();

        uint32 pending(void);
        uint32 available(void);

        uint32 read(void *buf, uint32 len);
        uint8  read(void);

        uint32 write(uint8);
        uint32 write(const char *str);
        uint32 write(void *, uint32);

        // Print.h functions with return values
        uint32 print(char);
        uint32 print(const char[]);
        uint32 print(uint8);
        uint32 print(int);
        uint32 print(unsigned int);
        uint32 print(long);
        uint32 print(unsigned long);
        uint32 print(long, int);
        uint32 print(double);
        uint32 println(void);
        uint32 println(char);
        uint32 println(const char[]);
        uint32 println(uint8);
        uint32 println(int);
        uint32 println(unsigned int);
        uint32 println(long);
        uint32 println(unsigned long);
        uint32 println(long, int);
        uint32 println(double);
};

extern USBSerial SerialUSB;

#endif

