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

int32 USBSerial::writeBlocking(uint8 ch) {
  uint32 status = 0;
  while(status < 1) {
    status = usbSendBytes(&ch, 1);
  }
  return status;
}

int32 USBSerial::writeBlocking(const char *str) {
  uint32 status = sent = 0;
  uint32 len = strlen(str);
  while(len > sent) {
    status = usbSendBytes((uint8*)str, len);
    if(status < 0) {
        return status;
    }
    sent += status;
  }
  return sent;
}

int32 USBSerial::writeBlocking(void *buf, uint32 size) {
  // buffer already check for null in write()
  uint32 status = sent = 0;
  while(size > sent) {
    status = usbSendBytes((uint8*)buf, size);
    if(status < 0) {
        return status;
    }
    sent += status;
  }
  return sent;
}

int32 USBSerial::writeNonBlocking(uint8 ch) {
  return usbSendBytes(&ch, 1);
}

int32 USBSerial::writeNonBlocking(const char *str) {
   uint32 len = strlen(str);
   return usbSendBytes((uint8*)str, len);
}

int32 USBSerial::writeNonBlocking(void *buf, uint32 size) {
   // buffer already check for null in write()
   return usbSendBytes((uint8*)buf, size);
}

int32 USBSerial::write(uint8 ch) {
    switch(this->timeout) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return writeNonBlocking(ch);
        case BLOCKING:
            return writeBlocking(ch);
        default:
            uint32 start = millis();
            int32 ret = 0;
            while((millis() - start < this->timeout) && ret == 0) {
                ret = writeNonBlocking(ch);
            }
            return ret;
    }
}

int32 USBSerial::write(const char *str) {
    switch(this->timeout) {
        case USB_DISABLED:
            return -1;
        case NONBLOCKING:
            return writeNonBlocking(str);
        case BLOCKING:
            return writeBlocking(str);
        default:
            uint32 len = strlen(str);
            uint32 start = millis();
            int32 ret = sent = 0;
            while((millis() - start < this->timeout) && sent < len) {
                ret = writeNonBlocking(str);
                if(ret < 0) {
                    return ret;
                }
                sent += ret;
            }
            return sent;
    }

}

int32 USBSerial::write(void *buf, uint32 size) {
   if (!buf) {
      return;
   }
   switch(this->timeout) {
       case USB_DISABLED:
           return -1;
       case NONBLOCKING:
           return writeNonBlocking(buf,size);
       case BLOCKING:
           return writeBlocking(buf,size);
       default:
            uint32 start = millis();
            int32 ret = sent = 0;
            while((millis() - start < this->timeout) && sent < size) {
                ret = writeNonBlocking(buf,size);
                if(ret < 0) {
                    return ret;
                }
                sent += ret;
            }
            return sent;
   }
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

USBSerial SerialUSB;

