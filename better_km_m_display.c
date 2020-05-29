/**********************************************************
 *
 * Milestone1Final
 *
 *Milestone 1 project,
 *Displaying raw, g and ms data from the accelerometer
 *on the OLED screen with reference orientation changing.
 *
 *
 *    S.Garvey, B.Petrie, C.Tichborne
 *    With credit to C.Moore and P.J.Bones
 *    16.03.20
 *
 **********************************************************/
//Including h files needed
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"
#include "acc.h"
#include "i2c_driver.h"
#include "buttons4.h"

/*Creating an x,y,z struct*/
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
uint16_t distance_flag = 0;
uint16_t stepcount_flag = 1;
uint16_t start_flag = 0;
uint16_t steps_flag = 0;
uint16_t step_counter = 0;
uint16_t kilometeres = 0;
uint16_t miles = 0;
uint16_t distance_counter = 0;
uint8_t mile_flag = 0;
uint8_t kilometre_flag = 1;
uint16_t kilometres = 0;

/*******************************************
 *      Local prototypes
 *******************************************/
void initClock (void);
void initDisplay (void);
void displayUpdate (char *str1, int16_t num, uint8_t charLine);
void displayUpdate2 (char *str1, int16_t num1, char *str2, char *str3, char *str4, uint8_t charLine);
void initAccl (void);
vector3_t getAcclData (void);

/***********************************************************
 * Initialisation functions: clock, SysTick, PWM, accl
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

//*****************************************************************************
// Function to display a changing message on the display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
void
displayUpdate (char *str1, int16_t num, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %3d", str1, num);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

void
displayUpdate2 (char *str1, int16_t num1, char *str2, char *str3, char *str4, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %2d %s %s %s", str1, num1, str2, str3, str4);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

/******************************************************************************
 * Function to read accelerometer, and reset reference orientation as needed
 ******************************************************************************/
vector3_t
getAcclData (void)
{
    //Setting up variables and vectors.
    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    uint8_t bytesToRead = 6;
    uint8_t buttonState;
    vector3_t ref_value;
    vector3_t data;
    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    //Placing accelerometer data into the vector for x,y and z
    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    //Checking the button state and then setting the reference orientation accordingly
    buttonState = checkButton (DOWN);
    updateButtons();
    switch (buttonState)
    {
    case PUSHED:
        ref_value = acceleration;
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;
    }
    //Initialising the reference orientation each time the board is reset
    if(start_flag == 0) {
        ref_value = acceleration;
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;

        start_flag = 1;
    }

    //Setting the new values of the accelerometer data to refelct the new reference orientation.
    data.x = acceleration.x - ref_x;
    data.y = acceleration.y - ref_y;
    data.z = acceleration.z - ref_z;

    return data;
}

/*******************************************
 * Calculating the norm of the acceleration
 ********************************************/
uint16_t calculate_norm (void)
{
    vector3_t g_accel;
        uint16_t norm =0;

        g_accel = getAcclData();
        g_accel.x = (1000 * g_accel.x) / 256;
        g_accel.y = (1000 * g_accel.y) / 256;
        g_accel.z = (1000 * g_accel.z) / 256;

        norm = sqrt((g_accel.x * g_accel.x)+ (g_accel.y * g_accel.y)+(g_accel.z * g_accel.z));

        return norm;
}

/***************************************************************
 * Finding the number of steps and the total distance in metres
 ******************************************************************/
uint16_t get_steps (void)
{
    uint16_t norm1 = calculate_norm();

    if (steps_flag == 0) {
          if(norm1 > 150) {
              steps_flag = 1;
              step_counter = step_counter + 1;
          }
    }
    if(norm1 < 150) {
        steps_flag = 0;
    }
    return step_counter;
}

uint16_t get_metres (void)
{
    uint16_t norm2 = calculate_norm();
    uint16_t metres = 0.9*distance_counter;

     if (steps_flag == 0) {
           if(norm2 > 150) {
               steps_flag = 1;
               distance_counter = distance_counter + 1;
           }
     }
     if(norm2 < 150) {
         steps_flag = 0;
     }


    if(metres >= 1000){
        distance_counter = 0;
    }

    return metres;

}

uint16_t get_kilometres (void)
{
    uint16_t metre_data = get_metres();
    if(metre_data >= 1000){
        kilometres = kilometres + 1;
    }

    return kilometres;
}

uint16_t get_miles (void)
{
    uint16_t miles;
    uint16_t data = get_metres();
    miles = data/1609;
    return miles;
}

uint16_t get_milimiles(void)
{
    uint16_t milimiles = get_kilometres() * 1609;
    return milimiles;
}

void change_state_distance (void) {
    uint8_t butState;
    butState = checkButton(LEFT);
    updateButtons();
    switch (butState)
    {
    case PUSHED:
        if (stepcount_flag == 1) {
            stepcount_flag = 0;
            distance_flag = 1;
        } else if (distance_flag == 1) {
            distance_flag = 1;
            stepcount_flag = 0;
            break;
        }

    }
}

void change_state_steps (void) {
    uint8_t butState;
    butState = checkButton(RIGHT);
    updateButtons();
    switch (butState)
    {
    case PUSHED:
        if (stepcount_flag == 1) {
            stepcount_flag = 1;
            distance_flag = 0;
        } else if (stepcount_flag == 0) {
            distance_flag = 0;
            stepcount_flag = 1;
            break;
        }

    }
}

void change_state_measurements (void) {
    uint8_t butState;
    butState = checkButton(UP);
    updateButtons();
    switch (butState)
    {
    case PUSHED:
        if(kilometre_flag == 1) {
            mile_flag  = 1;
            kilometre_flag = 0;
        } else if (mile_flag == 1) {
            mile_flag = 0;
            kilometre_flag = 1;
            break;
        }
    }
}

char* metre_string(void)
{
    char* result = malloc(4);
    char* metre_string;
    uint16_t metre_data = get_metres();
    sprintf(result, "%3d", metre_data);
    getchar();
    if(metre_data <= 9){
        metre_string = strcat("00", result);
    } else if(metre_data <= 99){
        metre_string = strcat("0", result);
    } else if(metre_data <=999){
        metre_string = result;
    }
    return metre_string;
}

/********************************************************
 * main
 ********************************************************/
int
main (void)
{
    uint16_t data;
    char* second_data;
    char* data_name;

    initClock ();
    initAccl ();
    initDisplay ();
    int i = 0;
    initButtons ();



    OLEDStringDraw ("Accelerometer", 0, 0);

    //While loop to display data depending on the change state function
    while (1)
    {
        change_state_steps();
        change_state_distance();
        change_state_measurements();
        SysCtlDelay (SysCtlClockGet () / 100);
        if (i == 16) {
          if (stepcount_flag == 1) {
              data = get_steps();
              data_name = "Steps: ";
              displayUpdate (data_name, data, 2);


        } if(distance_flag == 1) {
            if(kilometre_flag == 1) {
                data = get_kilometres();
                second_data = metre_string();
                displayUpdate2 ("D:", data, ".", second_data, "km", 2);

            }  else if (mile_flag == 1) {
                data = get_miles();
                second_data = metre_string();
                displayUpdate2 ("D:" ,data, ".", second_data, "mi", 2);

            }


       }
        i = 0;
      }
       i++;
    }
}
