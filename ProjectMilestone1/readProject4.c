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
#include "i2c_driver.h"
#include "buttons4.h"

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

uint32_t ref_x = 0;
uint32_t ref_y = 0;
uint32_t ref_z = 0;
uint16_t raw_flag = 1;
uint16_t g_flag = 0;
uint16_t ms_flag = 0;
uint16_t start_flag = 0;

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

    uint8_t buttonState;
    vector3_t ref_value;
    vector3_t data;

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

   // vector3_t data;
    ref_value = acceleration;

    buttonState = checkButton (DOWN);
    updateButtons();
    switch (buttonState)
    {
    case PUSHED:
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;
    }
    if(start_flag == 0) {
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;

        start_flag = 1;
    }
    data.x = acceleration.x - ref_x;
    data.y = acceleration.y - ref_y;
    data.z = acceleration.z - ref_z;

    return data;
}

vector3_t convert_to_ms (void)
{
    vector3_t ms_accel;
    ms_accel = getAcclData();
    ms_accel.x = (9810 * ms_accel.x) / 256;
    ms_accel.y = (9810 * ms_accel.y) / 256;
    ms_accel.z = (9810 * ms_accel.z) /256;
    return ms_accel;
}

vector3_t convert_to_g (void)
{
    vector3_t g_accel;
    g_accel = getAcclData();
    g_accel.x = (1000 * g_accel.x) / 256;
    g_accel.y = (1000 * g_accel.y) / 256;
    g_accel.z = (1000 * g_accel.z) / 256;
    return g_accel;
}


/*void switch_between (void) {

    if(buttonpuush) {
        if(data_1) {
            data2;
        } elif (data2) {
            data3;
        } elif (data3) {
            data1;
        }
    }

}*/

/*vector3_t initOrientation (void)
{
    uint8_t buttonState;
    uint8_t start_flag = 0;
    vector3_t ref_value;
    vector3_t numbers;
   // vector3_t data;
    ref_value = getAcclData();
    numbers = getAcclData();

    butState = checkButton (DOWN);
    updateButtons();
    switch (buttonState)
    {
    case PUSHED:
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;
    }
    if(start_flag == 0) {
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;

        start_flag = 1;
    }
    data.x = numbers.x - ref_x;
    data.y = numbers.y - ref_y;
    data.z = numbers.z - ref_z;

    return data;
}
*/
//vector3_t initOrientation (void)
//{


//}

void reset_prog (void) {

}

void change_state (void) {
    uint8_t butState;
    butState = checkButton(UP);
    updateButtons();
    switch (butState)
    {
    case PUSHED:
        if (raw_flag == 1) {
            raw_flag = 0;
            g_flag = 1;
        } else if (g_flag == 1) {
            g_flag = 0;
            ms_flag = 1;
        } else if (ms_flag == 1) {
            ms_flag = 0;
            raw_flag = 1;
            break;
        }
       // SysCtlDelay (SysCtlClockGet () / 150);
    }
}

/********************************************************
 * main
 ********************************************************/
int
main (void)
{
    vector3_t data;

    initClock ();
    initAccl ();
    initDisplay ();
    int i = 0;
    //initOrientation();
    initButtons ();
    //button_press();

   // vector3_t display_value;

    OLEDStringDraw ("Accelerometer", 0, 0);

    while (1)
    {
        change_state();
        SysCtlDelay (SysCtlClockGet () / 100);    // Approx 2 Hz

        if (raw_flag == 1) {

            data = getAcclData();

        }
        if(g_flag == 1) {
            data = convert_to_g();

        }
        if(ms_flag == 1) {
            data = convert_to_ms();

        }

       //display_value.x = data.x;// - ref_x;
      // display_value.y = data.y;// - ref_y;
     //  display_value.z = data.z;// - ref_z;

        if(i == 16) {
            displayUpdate ("Accl", "X", data.x, 1);
            displayUpdate ("Accl", "Y", data.y, 2);
            displayUpdate ("Accl", "Z", data.z, 3);
            i = 0;
        }
        i++;
    }
}
