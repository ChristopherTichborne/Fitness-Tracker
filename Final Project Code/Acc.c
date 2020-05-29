/**********************************************************
 *
 * readAcc.c
 *
 * Example code which reads acceleration in
 * three dimensions and displays the reulting data on
 * the Orbit OLED display.
 *
 *    C. P. Moore
 *    11 Feb 2020
 *
 **********************************************************/

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

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;

/**********************************************************
 * Constants
 **********************************************************/
// Systick configuration
#define SYSTICK_RATE_HZ    10
#define BUF_SIZE 10
#define SAMPLE_RATE_HZ 10

//*****************************************************************************
// Global variables
//*****************************************************************************
static circBuf_t g_inBuffer;   // Buffer of size BUF_SIZE integers (sample values)
static circBuf_t h_inBuffer;
static circBuf_t i_inBuffer;
//static uint32_t g_ulSampCnt;    // Counter for the interrupts

//*****************************************************************************

/*******************************************
 *      Local prototypes
 *******************************************/
void initClock (void);
void initDisplay (void);
void displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine);
void initAccl (void);
vector3_t getAcclData (void);

/***********************************************************
 * Initialisation functions: clock, SysTick, PWM
 ***********************************************************
 * Clock
 ***********************************************************/
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}

/*********************************************************
 * initDisplay
 *********************************************************/
void
initDisplay (void)
{
    // Initialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
// Function to display a changing message on the display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
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

/*********************************************************
 * initAccl
 *********************************************************/
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

    // set +-16g, 13 bit resolution, active low interrupts
    toAccl[0] = ACCL_DATA_FORMAT;
    toAccl[1] = (ACCL_RANGE_16G | ACCL_FULL_RES);
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
    vector3_t mean_accel;
    uint8_t bytesToRead = 6;
    int16_t i = 0;
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;

    int16_t mean_x = 0;
    int16_t mean_y = 0;
    int16_t mean_z = 0;

    initCircBuf (&g_inBuffer, BUF_SIZE);
    initCircBuf (&h_inBuffer, BUF_SIZE);
    initCircBuf (&i_inBuffer, BUF_SIZE);

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];


    while (i < BUF_SIZE)
        {
        writeCircBuf (&g_inBuffer, acceleration.x);
        writeCircBuf (&h_inBuffer, acceleration.y);
        writeCircBuf (&i_inBuffer, acceleration.z);
        i++;
        }
    i = 0;

    while (i < BUF_SIZE)
    {
            sum_x = sum_x + readCircBuf (&g_inBuffer);
            sum_y = sum_y + readCircBuf (&h_inBuffer);
            sum_z = sum_z + readCircBuf (&i_inBuffer);
            i++;
    }

    mean_x = sum_x / BUF_SIZE;
    mean_y = sum_y / BUF_SIZE;
    mean_z = sum_z / BUF_SIZE;

    // Calculate and display the rounded mean of the buffer contents
    mean_accel.x = mean_x; // Return 16-bit acceleration readings.
    mean_accel.y = mean_y;
    mean_accel.z = mean_z;


    return mean_accel;

}


/********************************************************
 * main
 ********************************************************/
int
main (void)
{
    vector3_t acceleration_raw;

    initClock ();
    initAccl ();
    initDisplay ();

    OLEDStringDraw ("Accelerometer", 0, 0);


    while (1)
    {
        SysCtlDelay (SysCtlClockGet () / 6);    // Approx 2 Hz
        acceleration_raw = getAcclData();
        displayUpdate ("Accl", "X", acceleration_raw.x, 1);
        displayUpdate ("Accl", "Y", acceleration_raw.y, 2);
        displayUpdate ("Accl", "Z", acceleration_raw.z, 3);

    }
}
