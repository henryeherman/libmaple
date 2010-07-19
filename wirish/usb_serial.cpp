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

#include <string.h>

#include "wirish.h"
#include "usb.h"

#define DEFAULT_USB_TIMEOUT     1

USBSerial::USBSerial(void) {
    this->mode = USB_DISABLED;
}

void USBSerial::begin(void) {
    begin(TIMEOUT,DEFAULT_USB_TIMEOUT);
}

void USBSerial::begin(uint8 mode) {
    begin(mode,DEFAULT_USB_TIMEOUT);
}

void USBSerial::begin(uint8 mode, uint32 timeout) {
    this->mode = mode;
    this->timeout = timeout;
    setupUSB();
}

void USBSerial::end() {
  this->mode = USB_DISABLED;
  //disableUSB();   //TODO: implement
}

uint32 USBSerial::write(uint8 ch) {
    uint32 status = 0;
    switch(this->mode) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes(&ch, 1);
        case BLOCKING:
            while(status == 0) {
                status = usbSendBytes(&ch, 1);
            }
            return status;
        case TIMEOUT:
            uint32 start = millis();
            while(status == 0 && (millis() - start) <= this->timeout) {
                status = usbSendBytes(&ch, 1);
            }
            return status;
    }
    return 0;
}

uint32 USBSerial::write(const char *str) {
    uint32 status = 0;
    uint32 len = strlen(str);
    switch(this->mode) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes((uint8*)str, len);
        case BLOCKING:
            while(len > status) {
                status += usbSendBytes((uint8*)str+status, len-status);
            }
            return status;
        case TIMEOUT:
            uint32 start = millis();
            uint32 oldstatus = 0;
            while(len > status && (millis() - start) <= this->timeout) {
                status += usbSendBytes((uint8*)str+status, len-status);
                if(status != oldstatus) {
                    start = millis();
                    oldstatus = status;
                }
            }
            return status;
    }
    return 0;
}

uint32 USBSerial::write(void *buf, uint32 size) {
    if (!buf) {
        return 0;
    }
    uint32 status = 0;
    switch(this->mode) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes((uint8*)buf, size);
        case BLOCKING:
            while(size > status) {
                status += usbSendBytes((uint8*)buf, size);
            }
            return status;
        case TIMEOUT:
            uint32 start = millis();
            uint32 oldstatus = 0;
            while(size > status && (millis() - start) <= this->timeout) {
                status += usbSendBytes((uint8*)buf+status, size-status);
                if(status != oldstatus) {
                    start = millis();
                    oldstatus = status;
                }
            }
            return status;
    }
    return 0;
}

uint32 USBSerial::available(void) {
   return usbBytesAvailable();
}

uint32 USBSerial::pending(void) {
   return usbGetCountTx();
}

uint32 USBSerial::read(void *buf, uint32 len) {
   if (!buf) {
      return 0;
   }
   return usbReceiveBytes((uint8*)buf, len);
}

uint8 USBSerial::read(void) {
   uint8 ch;
   usbReceiveBytes(&ch, 1);
   return ch;
}

// Print.h functions with return values...

uint32 USBSerial::print(uint8 b) {
  return write(b);
}

uint32 USBSerial::print(char c) {
  return write((byte) c);
}

uint32 USBSerial::print(const char str[]) {
  return write(str);
}

uint32 USBSerial::print(int n) {
  return print((long) n);
}

uint32 USBSerial::print(unsigned int n) {
  return print((unsigned long) n);
}

uint32 USBSerial::print(long n) {
  uint32 status = print('-');
  return status + printNumber(n, 10);
}

uint32 USBSerial::print(unsigned long n) {
  return printNumber(n, 10);
}

uint32 USBSerial::print(long n, int base) {
  if (base == 0)
    return print((char) n);
  else if (base == 10)
    return print(n);
  else
    return printNumber(n, base);
}

uint32 USBSerial::print(double n) {
  return printFloat(n, 2);
}

uint32 USBSerial::println(void) {
  return write("\r\n");
}

uint32 USBSerial::println(char c) {
  uint32 status = print(c);
  return status + println();
}

uint32 USBSerial::println(const char c[]) {
  uint32 status = print(c);
  return status + println();
}

uint32 USBSerial::println(uint8 b) {
  uint32 status = print(b);
  return status + println();
}

uint32 USBSerial::println(int n) {
  uint32 status = print(n);
  return status + println();
}

uint32 USBSerial::println(unsigned int n) {
  uint32 status = print(n);
  return status + println();
}

uint32 USBSerial::println(long n) {
  uint32 status = print(n);
  return status + println();
}

uint32 USBSerial::println(unsigned long n) {
  uint32 status = print(n);
  return status + println();
}

uint32 USBSerial::println(long n, int base) {
  uint32 status = print(n,base);
  return status + println();
}

uint32 USBSerial::println(double n) {
  uint32 status = print(n);
  return status + status;
}

// Private Methods /////////////////////////////////////////////////////////////

uint32 USBSerial::printNumber(unsigned long n, uint8 base) {
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;
  int32 status = 0;

  if (n == 0) {
    return print('0');
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--) {
      status += print((char) (buf[i - 1] < 10 ?  '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
  }
  return status;
}

uint32 USBSerial::printFloat(double number, uint8 digits) {
  uint32 status = 0;
  // Handle negative numbers
  if (number < 0.0)
  {
     status += print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8 i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  status += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    status += print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    status += print(toPrint);
    remainder -= toPrint;
  }
  return status;
}

USBSerial SerialUSB;

