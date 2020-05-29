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
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"


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
static uint32_t g_ulSampCnt;    // Counter for the interrupts

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
SysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;
}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
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


/*
void
ADCIntHandler(void)
{
    uint32_t g_ulValue;
    uint32_t h_ulValue;
    uint32_t i_ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &g_ulValue);
    ADCSequenceDataGet(ADC0_BASE, 3, &h_ulValue);
    ADCSequenceDataGet(ADC0_BASE, 3, &i_ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    writeCircBuf (&h_inBuffer, ulValue);
    writeCircBuf (&i_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}
*/

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



int main (void)

{

    initClock ();
    initAccl ();
    initDisplay ();

    OLEDStringDraw ("Accelerometer", 0, 0);

    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    //vector3_t acceleration;
    uint8_t bytesToRead = 6;
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initCircBuf (&h_inBuffer, BUF_SIZE);
    initCircBuf (&i_inBuffer, BUF_SIZE);

 //   vector3_t acceleration;

   // uint32_t g_ulValue;
    //uint32_t h_ulValue;
    //uint32_t i_ulValue;
    int sum_x;
    int sum_y;
    int sum_z;

    //ADCSequenceDataGet(ADC0_BASE, 3, &g_ulValue);
    //ADCSequenceDataGet(ADC0_BASE, 3, &h_ulValue);
    //ADCSequenceDataGet(ADC0_BASE, 3, &i_ulValue);

    writeCircBuf (&g_inBuffer, sum_x);
    writeCircBuf (&h_inBuffer, sum_y);
    writeCircBuf (&i_inBuffer, sum_z);

    //ADCIntClear(ADC0_BASE, 3);

    //fromAccl[0] = ACCL_DATA_X0;
    //I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);
    int i = 0;
    //acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    //acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    //acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    while(i < 20)
    {
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;

        for(i=0; i<BUF_SIZE;i++) {
            sum_x = sum_x + readCircBuf(&g_inBuffer);
            sum_y = sum_y + readCircBuf(&h_inBuffer);
            sum_z = sum_z + readCircBuf(&i_inBuffer);
        }
        SysCtlDelay (SysCtlClockGet () / 6);    // Approx 2 Hz
        //acceleration_raw = getAcclData();

        /*
        displayMeanVal((2* sum_x + BUF_SIZE)/2/BUF_SIZE, g_ulSampCnt);
        displayMeanVal((2* sum_x + BUF_SIZE)/2/BUF_SIZE, g_ulSampCnt);
        displayMeanVal((2* sum_x + BUF_SIZE)/2/BUF_SIZE, g_ulSampCnt);
        SysCtlDelay(SysCtlClockGet()/6);
        */
    }
    while(1)
    {
        displayUpdate ("Accl", "X", sum_x, 1);
        displayUpdate ("Accl", "Y", sum_y, 2);
        displayUpdate ("Accl", "Z", sum_z, 3);
    }
   // return acceleration;
}


/*
vector3_t
getAcclData (void)
{

    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    vector3_t mean_accel;
    uint8_t bytesToRead = 6;
    int i = 0;
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initCircBuf (&h_inBuffer, BUF_SIZE);
    initCircBuf (&i_inBuffer, BUF_SIZE);

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];


    while (1)
        {
        writeCircBuf (&g_inBuffer, acceleration.x);
        writeCircBuf (&h_inBuffer, acceleration.y);
        writeCircBuf (&i_inBuffer, acceleration.z);

        //
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and display it, together with the sample number.

        int sum_x = 0;
        int sum_y = 0;
        int sum_z = 0;

        int mean_x = 0;
        int mean_y = 0;
        int mean_z = 0;

        if(i == 10)
            {

            i = 0;

            sum_x = sum_x + readCircBuf (&g_inBuffer);
            sum_y = sum_y + readCircBuf (&h_inBuffer);
            sum_z = sum_z + readCircBuf (&i_inBuffer);

            mean_x = sum_x / BUF_SIZE;
            mean_y = sum_y / BUF_SIZE;
            mean_z = sum_z / BUF_SIZE;

            // Calculate and display the rounded mean of the buffer contents
            mean_accel.x = (mean_x); // Return 16-bit acceleration readings.
            mean_accel.y = (mean_y);
            mean_accel.z = (mean_z);
            return mean_accel;

            } else {

                i++;
            }


        }

}
*/

/********************************************************
 * main
 ********************************************************/
/*

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
*/
