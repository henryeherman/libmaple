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

USBSerial::USBSerial(void) {
    this->timeout = USB_DISABLED;
}

void USBSerial::begin(void) {
    begin(10);
}

void USBSerial::begin(int32 mode) {
    if(mode < -2) {
        mode = -2;
    }
    this->timeout = mode;
    setupUSB();
}

void USBSerial::end() {
  this->timeout = USB_DISABLED;
  //disableUSB();   //TODO: implement
}

int32 USBSerial::write(uint8 ch) {
    int32 status = 0;
    switch(this->timeout) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes(&ch, 1);
        case BLOCKING:
            while(status == 0) {
                status = usbSendBytes(&ch, 1);
            }
            return status;
        default:
            uint32 start = millis();
            while(status == 0 && ((int32)(millis() - start) < this->timeout)) {
                status = usbSendBytes(&ch, 1);
            }
            return status;
    }
}

int32 USBSerial::write(const char *str) {
    int32 status = 0;
    int32 sent = 0;
    int32 len = strlen(str);
    switch(this->timeout) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes((uint8*)str, len);
        case BLOCKING:
            while(len > sent) {
                status = usbSendBytes((uint8*)str, len);
                if(status < 0) {
                    return status;
                }
                sent += status;
            }
            return sent;
        default:
            int32 start = millis();
            while(len > sent && ((int32)(millis() - start) < this->timeout)) {
                status = usbSendBytes((uint8*)str, len);
                if(status < 0) {
                    return status;
                }
                sent += status;
            }
            return sent;
    }
}

int32 USBSerial::write(void *buf, uint32 size) {
    if (!buf) {
        return 0;
    }
    uint32 sent = 0;
    uint32 status = 0;
    switch(this->timeout) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return usbSendBytes((uint8*)buf, size);
        case BLOCKING:
            while(size > sent) {
                status = usbSendBytes((uint8*)buf, size);
                if(status < 0) {
                    return status;
                }
                sent += status;
            }
            return sent;
        default:
            uint32 start = millis();
            while(size > sent && ((int32)(millis() - start) < this->timeout)) {
                status = usbSendBytes((uint8*)buf, size);
                if(status < 0) {
                    return status;
                }
                sent += status;
            }
            return sent;
    }
}

uint32 USBSerial::available(void) {
   return usbBytesAvailable();
}

uint8 USBSerial::pending(void) {
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

int32 USBSerial::print(uint8 b) {
  return write(b);
}

int32 USBSerial::print(char c) {
  return print((byte) c);
}

int32 USBSerial::print(const char str[]) {
  return write(str);
}

int32 USBSerial::print(int n) {
  return print((long) n);
}

int32 USBSerial::print(unsigned int n) {
  return print((unsigned long) n);
}

int32 USBSerial::print(long n) {
  int32 ret = 0;
  if (n < 0) {
    ret += print('-');
    n = -n;
  }
  ret += printNumber(n, 10);
  return ret;
}

int32 USBSerial::print(unsigned long n) {
  return printNumber(n, 10);
}

int32 USBSerial::print(long n, int base) {
  if (base == 0)
    return print((char) n);
  else if (base == 10)
    return print(n);
  else
    return printNumber(n, base);
}

int32 USBSerial::print(double n) {
  return printFloat(n, 2);
}

int32 USBSerial::println(void) {
  return print("\r\n");
}

int32 USBSerial::println(char c) {
  if(print(c)) {
    return println() + 1;
  } else {
    return -1;
  }
}

int32 USBSerial::println(const char c[]) {
  return print(c) + println();
}

int32 USBSerial::println(uint8 b) {
  return print(b) + println();
}

int32 USBSerial::println(int n) {
  return print(n) + println();
}

int32 USBSerial::println(unsigned int n) {
  return print(n) + println();
}

int32 USBSerial::println(long n) {
  return print(n) + println();
}

int32 USBSerial::println(unsigned long n) {
  return print(n) + println();
}

int32 USBSerial::println(long n, int base) {
  return print(n, base) + println();
}

int32 USBSerial::println(double n) {
  return print(n) + println();
}

// Private Methods /////////////////////////////////////////////////////////////

int32 USBSerial::printNumber(unsigned long n, uint8 base) {
  unsigned char buf[8 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long i = 0;
  int32 ret = 0;

  if (n == 0) {
    return print('0');
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--) {
      ret += print((char) (buf[i - 1] < 10 ?  '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));
  }
  return ret;
}

int32 USBSerial::printFloat(double number, uint8 digits) {
  int32 ret = 0;
  // Handle negative numbers
  if (number < 0.0)
  {
     ret += print('-');
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
  ret += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    ret += print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    ret += print(toPrint);
    remainder -= toPrint;
  }
  return ret;
}

USBSerial SerialUSB;

