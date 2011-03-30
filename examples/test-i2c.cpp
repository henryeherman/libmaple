
#include "wirish.h"
#include "i2c.h"


#define I2C_SLAVE  1
#define I2C_MASTER 2

//HardwareUsb usb;


/* ADXL Defines */
#define ADXL_ADDR	0xA6

//ADXL Register Map
#define	DEVID			0x00	//Device ID Register
#define THRESH_TAP		0x1D	//Tap Threshold
#define	OFSX			0x1E	//X-axis offset
#define	OFSY			0x1F	//Y-axis offset
#define	OFSZ			0x20	//Z-axis offset
#define	DUR				0x21	//Tap Duration
#define	Latent			0x22	//Tap latency
#define	Window			0x23	//Tap window
#define	THRESH_ACT		0x24	//Activity Threshold
#define	THRESH_INACT	0x25	//Inactivity Threshold
#define	TIME_INACT		0x26	//Inactivity Time
#define	ACT_INACT_CTL	0x27	//Axis enable control for activity and inactivity detection
#define	THRESH_FF		0x28	//free-fall threshold
#define	TIME_FF			0x29	//Free-Fall Time
#define	TAP_AXES		0x2A	//Axis control for tap/double tap
#define ACT_TAP_STATUS	0x2B	//Source of tap/double tap
#define	BW_RATE			0x2C	//Data rate and power mode control
#define POWER_CTL		0x2D	//Power Control Register
#define	INT_ENABLE		0x2E	//Interrupt Enable Control
#define	INT_MAP			0x2F	//Interrupt Mapping Control
#define	INT_SOURCE		0x30	//Source of interrupts
#define	DATA_FORMAT		0x31	//Data format control
#define DATAX0			0x32	//X-Axis Data 0
#define DATAX1			0x33	//X-Axis Data 1
#define DATAY0			0x34	//Y-Axis Data 0
#define DATAY1			0x35	//Y-Axis Data 1
#define DATAZ0			0x36	//Z-Axis Data 0
#define DATAZ1			0x37	//Z-Axis Data 1
#define	FIFO_CTL		0x38	//FIFO control
#define	FIFO_STATUS		0x39	//FIFO status

//Power Control Register Bits
#define WU_0		(1<<0)	//Wake Up Mode - Bit 0
#define	WU_1		(1<<1)	//Wake Up mode - Bit 1
#define SLEEP		(1<<2)	//Sleep Mode
#define	MEASURE		(1<<3)	//Measurement Mode
#define AUTO_SLP	(1<<4)	//Auto Sleep Mode bit
#define LINK		(1<<5)	//Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define	OVERRUN		(1<<0)
#define	WATERMARK	(1<<1)
#define FREE_FALL	(1<<2)
#define	INACTIVITY	(1<<3)
#define	ACTIVITY	(1<<4)
#define DOUBLE_TAP	(1<<5)
#define	SINGLE_TAP	(1<<6)
#define	DATA_READY	(1<<7)

//Data Format Bits
#define RANGE_0		(1<<0)
#define	RANGE_1		(1<<1)
#define JUSTIFY		(1<<2)
#define	FULL_RES	(1<<3)

#define	INT_INVERT	(1<<5)
#define	SPI		(1<<6)
#define	SELF_TEST	(1<<7)

int ledPin = 13;
int toggle = 1;

uint8 data[10];
uint8 command[2];

void setupADX345() {

    // Put ACCL in Measure mode
    data[0] = POWER_CTL;
    data[1] = MEASURE;
    i2c_master_write(I2C_PORT1, ADXL_ADDR, data, 2);

    delay(100);
    // Set dynamic range
    data[0] = DATA_FORMAT;
    data[1] = RANGE_0;
    i2c_master_write(I2C_PORT1, ADXL_ADDR, data, 2);

}

int readX() {
    uint8 value;
    uint8 command[2];
    command[0] = DATAX0;
    command[1] = DATAX1;
    
    SerialUSB.println("Reading Value");
    i2c_master_write(I2C_PORT1, ADXL_ADDR, command, 1);
    
    delay(10);
    
    i2c_master_read(I2C_PORT1, ADXL_ADDR, data, 1);
    
    
    //i2c_master_read(I2C_PORT1, ADXL_ADDR, DATAX1, &(data[1]), 1);
    
    value = data[0]; // + (data[1]<<8);
    return value;
}

int readY(){
    return 0;
}

int readZ(){
    return 0;
}

void setup() {
    pinMode(ledPin, OUTPUT);
    //data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = 100;
    i2c_init(I2C_PORT1, 50000);
    SerialUSB.begin();
    delay(1);
    SerialUSB.println("Setup Acc");
    setupADX345();
    SerialUSB.println("End Setup");

}

void loop() {
    command[0] = 0x32;
    uint8 value=0;
    digitalWrite(ledPin, 1);
    delay(500);
    digitalWrite(ledPin, 0);
    delay(500);
    SerialUSB.println("Loop");
    i2c_master_write(I2C_PORT1, ADXL_ADDR, command, 1);
    delay(50);
    i2c_master_read(I2C_PORT1, ADXL_ADDR, data, 1);
    value = data[0];
    SerialUSB.println(value,BYTE);
    
}


int main(void) {
    init();
    setup();

    while (1) {
        loop();
    }
    return 0;
}


