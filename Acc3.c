
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "../OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "acc.h"
#include "circBufT.h"
#include "i2c_driver.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;

#define SYSTICK_RATE_HZ    10
#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10

static circBuf_t x_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
//static uint32_t x_ulSampCnt;
static circBuf_t y_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
//static uint32_t y_ulSampCnt;
static circBuf_t z_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
//static uint32_t z_ulSampCnt;

void initClock (void);
void initDisplay (void);
void displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine);
void initAccl (void);
vector3_t getAcclData (void);

void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}
void
initDisplay (void)
{
    // Initialise the Orbit OLED display
    OLEDInitialise ();
}

void
displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %s %3d", str1, str2, num);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

void
initAccl (void)
{
    char    toAccl[] = {0, 0};  // parameter, value

    /*
     * Enable I2C Peripheral
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    /*
     * Set I2C GPIO pins
     */
    GPIOPinTypeI2C(I2CSDAPort, I2CSDA_PIN);
    GPIOPinTypeI2CSCL(I2CSCLPort, I2CSCL_PIN);
    GPIOPinConfigure(I2CSCL);
    GPIOPinConfigure(I2CSDA);

    /*
     * Setup I2C
     */
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    GPIOPinTypeGPIOInput(ACCL_INT2Port, ACCL_INT2);

    //Initialize ADXL345 Acceleromter

    // set +-2g, 10 bit resolution, active low interrupts
    toAccl[0] = ACCL_DATA_FORMAT;
    toAccl[1] = (ACCL_RANGE_2G | ACCL_FULL_RES);
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_PWR_CTL;
    toAccl[1] = ACCL_MEASURE;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);


    toAccl[0] = ACCL_BW_RATE;
    toAccl[1] = ACCL_RATE_100HZ;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_INT;
    toAccl[1] = 0x00;       // Disable interrupts from accelerometer.
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_X;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Y;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Z;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);
}

/********************************************************
 * Function to read accelerometer
 ********************************************************/
vector3_t
getAcclData (void)
{
    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    uint8_t bytesToRead = 6;

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    return acceleration;
}

vector3_t convert_to_ms (void)
{
    vector3_t data;
    data = getAcclData();
    data.x = (981 * data.x) / 256;
    data.y = (981 * data.y) / 256;
    data.z = (981 * data.z) /256;
    return data;
}

/********************************************************
 * main
 ********************************************************/

/*
void
displayMeanVal(uint16_t meanVal, uint32_t count)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("ADC demo 1", 0, 0);

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", meanVal);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "Sample # %5d", count);
    OLEDStringDraw (string, 0, 3);
}
*/
int
main (void)
{

    uint16_t i;
    int32_t sum_x;
    int32_t sum_y;
    int32_t sum_z;


    vector3_t acceleration_raw;

    initClock ();
    initAccl ();
    initDisplay ();
    initCircBuf (&x_inBuffer, BUF_SIZE);
    initCircBuf (&y_inBuffer, BUF_SIZE);
    initCircBuf (&z_inBuffer, BUF_SIZE);



    OLEDStringDraw ("Accelerometer", 0, 0);

    while (1)
    {
        SysCtlDelay (SysCtlClockGet () / 6);    // Approx 2 Hz
        //acceleration_raw = convert_to_ms();
        //displayUpdate ("Accl", "X", acceleration_raw.x, 1);
      //  displayUpdate ("Accl", "Y", acceleration_raw.y, 2);
        //displayUpdate ("Accl", "Z", acceleration_raw.z, 3);

        //
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number.

        for (i = 0; i < BUF_SIZE; i++)
        {
            sum_x = sum_x + readCircBuf (&x_inBuffer);
            sum_y = sum_y + readCircBuf (&y_inBuffer);
            sum_z = sum_z + readCircBuf (&z_inBuffer);
            // Calculate and display the rounded mean of the buffer contents
            //displayMeanVal ((2 * sum_x + BUF_SIZE) / 2 / BUF_SIZE, 2);
           //displayMeanVal ((2 * sum_y + BUF_SIZE) / 2 / BUF_SIZE, 2);
            //displayMeanVal ((2 * sum_z + BUF_SIZE) / 2 / BUF_SIZE, 2);


         SysCtlDelay (SysCtlClockGet() / 6);  // Update display at ~ 2 Hz
         displayUpdate ("Accl", "X", sum_x, 1);
         displayUpdate ("Accl", "Y", sum_y, 2);
         displayUpdate ("Accl", "Z", sum_z, 3);
        }


    }

}
