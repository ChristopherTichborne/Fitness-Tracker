/****************************************************************************
 *
 * ENCE361 Fitness Tracker Project
 *
 *    Final Version
 *
 *    A personal fitness monitor that tracks the steps taken and estimates
 *    distance traveled and includes a test mode.
 *
 *    S.Garvey, B.Petrie, C.Tichborne
 *    With credit to C.Moore and P.J.Bones
 *    16.03.20
 *
 ***************************************************************************/
//Including h files needed
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
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

/***************************************************************
 * Creating a x,y,z structure to use for the accelerometer data
 ***************************************************************/
typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;

/****************************************************************
 * Global Constants to be used throughout the code by any tasks
 ***************************************************************/
// Systick configuration
#define SYSTICK_RATE_HZ    10
uint32_t ref_x = 0;
uint32_t ref_y = 0;
uint32_t ref_z = 0;
uint16_t distance_flag = 0; // Is 1 when the OLED display is showing the distance moved
uint16_t stepcount_flag = 1; // Is 1 when the OLED display is showing the accumulated steps
uint16_t start_flag = 0; // Is used to decide if the program has restarted. Once the reference value is set, it is set to 1.
uint16_t steps_flag ; // Used to determine if a step has occurred
uint16_t step_counter;// Counts the number of times steps_flag is changed to 1 and back again to 0
uint16_t distance_counter;//""                                     ""                       ""
uint16_t metres;
uint16_t kilometres;
uint16_t miles;
uint16_t milimiles;
uint8_t mile_flag = 0; // Kilometre and mile flag are used to decide is the OLED is displaying distance in km or mi while distance_flag = 1.
uint8_t kilometre_flag = 1;
uint16_t norm = 0;
uint16_t start_time = 0;
uint16_t end_time = 0;
uint16_t total_time = 0;
uint8_t test_flag = 0; // When the test_flag is set to 1, the device is in test mode.


/*******************************************
 *      Local prototypes
 *******************************************/
void initClock (void);
void initDisplay (void);
void displayUpdate (char *str1, int16_t num, uint8_t charLine);
void displayUpdate2 (int16_t num1, char *str2, int16_t num2, char *str3, uint8_t charLine);
void initAccl (void);
vector3_t getAcclData (void);

/***********************************************************
 * Initialization functions: clock, SysTick, PWM, accl
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

    //Initialize ADXL345 Accelerometer

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
// Functions to display a changing message on the display.
// The display has 4 rows of 16 characters, with 0, 0 at top left.
//*****************************************************************************
/******************************************************************************
 * displayUpdate is used to display the step count
 ******************************************************************************/

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
/******************************************************************************
 * displayUpdate2 is used to display the distance traveled
 ******************************************************************************/

void
displayUpdate2 (int16_t num1, char *str2, int16_t num2, char *str3, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%2d %s %03d %s", num1, str2, num2, str3);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

/************************************************************************************
 * Function to read accelerometer, and reset reference orientation at each start up
 ************************************************************************************/
vector3_t
getAcclData (void)
{
    //Setting up variables and vectors.
    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    uint8_t bytesToRead = 6;
    //uint8_t buttonState;
    vector3_t ref_value;
    vector3_t data;
    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    //Placing accelerometer data into the vector for x,y and z
    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    //Initialising the reference orientation each time the board is reset
    if(start_flag == 0) {
        ref_value = acceleration;
        ref_x = ref_value.x;
        ref_y = ref_value.y;
        ref_z = ref_value.z;

        start_flag = 1;
    }

    //Setting the new values of the accelerometer data to reflect the start up reference orientation.
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


        g_accel = getAcclData();
        g_accel.x = (1000 * g_accel.x) / 256;
        g_accel.y = (1000 * g_accel.y) / 256;
        g_accel.z = (1000 * g_accel.z) / 256;

        norm = sqrt((g_accel.x * g_accel.x)+ (g_accel.y * g_accel.y)+(g_accel.z * g_accel.z));

        return norm;
}

/********************************************************************************
 * Finding the number of steps and the total distance in metres,
 * A step occurs each time the norm goes greater than 1.5G and then back below
 *********************************************************************************/
void get_steps (void)
{
    //Will not occur in test_mode
    if(test_flag == 0){
    //Checking that the user is not currently doing a step
    if (steps_flag == 0) {
          if(norm > 150) {
              steps_flag = 1; //When steps_flag is 1, the norm is greater than 1.5G
              step_counter = step_counter + 1; // Incrementing the counters used for steps and distance
              distance_counter = distance_counter + 1;

          }
    }
    if(norm < 150) {
        steps_flag = 0; //Another step will not be recorded until norm is under 1.5G
    }

    }
}
/**********************************************************************
 * Function to estimate the distance travelled in m, km, mi and mmi
 * A step has been assumed as equal to 0.9m
 **********************************************************************/
void get_distance (void)
{
    //All distances are based off the distance counter
    metres = 0.9*distance_counter;
    kilometres = 0.9*distance_counter /1000;
    miles = 0.9*distance_counter/1609;
    milimiles = 0.9*distance_counter/1.609;

   //The total value of metres/milimiles is subtracted, since metres/milimiles is based off the distance counter
   //Once distance counter is over 1000, it cannot be reset as both km and mi are found from it also. So metres and milimiles are constantly reset.
   if(metres >= 1000){
        metres = metres - kilometres*1000;
    }
   if(milimiles >= 1000) {
       milimiles = milimiles - miles*1000;
   }

}

/**********************************************************************
 * Setting the distance count flag for the OLED display
 **********************************************************************/
void change_state_distance (void) {
    uint8_t butState; //Variable to store the button state
    butState = checkButton(LEFT);//Storing the button state of the LEFT button at that time
    updateButtons();
    switch (butState)
    {
    case PUSHED:
        if (stepcount_flag == 1) {
            stepcount_flag = 0;
            distance_flag = 1;
        } else if (distance_flag == 1) { //If the display is already displaying the distance, a second push will not affect the display
            distance_flag = 1;
            stepcount_flag = 0;
            break;
        }

    }
}

/**********************************************************************
 * Setting the step count flag for the OLED display,
 * same code as change_state_distance but checking the RIGHT button
 **********************************************************************/
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

/*************************************************************************
 * Setting the kilometre and mile flag for changing the distance display
 ************************************************************************/
void change_state_measurements (void) {
    if(test_flag == 0)
    {
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
}

/******************************************************************************************
 * Slow button push to reset the distance or step counter depending on which flag is high
 ******************************************************************************************/
void reset_counter (void)
{
    if(test_flag == 0){
    uint8_t butState;
        butState = checkButton(DOWN);
        updateButtons();
        switch (butState)
        {
        case PUSHED:
        SysCtlDelay(SysCtlClockGet()/5);
        if(stepcount_flag == 1){
            step_counter = 0;
        } else if(distance_flag) {
            distance_counter = 0;

        }

        }


 }
}

/**********************************************************************
 * Checking the slide switch 1 to set the test mode flag
 **********************************************************************/
void test_flag_set (void)
{
    uint8_t switch_state;
    switch_state = checkButton(SLIDE);
    updateButtons();
    switch (switch_state)
    {
    case PUSHED:
        test_flag = 1;
    }
    switch (switch_state)
        {
        case RELEASED:
            test_flag = 0;
        }
}

/**********************************************************************
 * When in test mode, the up button will increment the distance
 * and step counter by 100 for each push
 **********************************************************************/
void test_mode_up (void)
{
   if(test_flag == 1)
   {
       uint8_t butState;
       butState = checkButton(UP);
       updateButtons();
       switch (butState)
       {
       case PUSHED:
           step_counter = step_counter + 100;
           distance_counter = distance_counter + 100;


    }
   }
}

/**********************************************************************
 * When in test mode, the down button will decrement the distance
 * and step counter by 500 for each push
 **********************************************************************/
void test_mode_down (void)
{
    if(test_flag == 1)
    {

        uint8_t butState;
        butState = checkButton(DOWN);
        updateButtons();
        switch (butState)
        {
        case PUSHED:
            if(step_counter <= 500)
            {
                step_counter = 0; //If the step counter is already less than 500, set to zero to avoid negative values
            } else
            {
                step_counter = step_counter - 500;
            }
            if(distance_counter <= 500)
            {
                distance_counter = 0; //If the distance counter is already less than 500, set to zero to avoid negative values
            } else {
                distance_counter = distance_counter - 500;
            }

    }

}
}

/********************************************************
 * Round Robin Scheduler used in Main Function
 ********************************************************/
int main (void)
{
    uint16_t data; //Setting variables to use in the displayUpdate functions
    uint16_t second_data;
    char* data_name;

    initClock ();
    initAccl ();
    initDisplay ();
    int i = 0; //Counter used to delay while loop
    initButtons ();





    //While loop to display data depending on the change state function
    while (1)
    {

        calculate_norm();  //Calling tasks that need to be checked each time
        change_state_steps();
        change_state_distance();
        change_state_measurements();
        reset_counter();
        test_flag_set();
        test_mode_up();
        test_mode_down();



        SysCtlDelay (SysCtlClockGet () / 100);
        if (i == 8) { //Counting to 8 allows averaging of the accelerometer data and also stabilizes the OLED display
            get_steps();
            get_distance();
            if(test_flag == 0){ // Tasks to be executed while not in test mode
                OLEDStringDraw ("Iso FitBit 2.0", 0, 0);
          if (stepcount_flag == 1) { //Display for steps
              data = step_counter;
              data_name = "Steps: ";
              displayUpdate (data_name, data, 2);
              OLEDStringDraw("            ", 0, 3);


        } if(distance_flag == 1) { //Display for distance
            OLEDStringDraw("Distance:  ", 0, 2);
            if(kilometre_flag == 1) { //Kilometres
                data = kilometres;
                second_data = metres;
                displayUpdate2 (data, ".", second_data, "km", 3); //Hard coding the decimal point for meters and miles

            }  else if (mile_flag == 1) { //Miles
                data = miles;
                second_data = milimiles;
                displayUpdate2 (data, ".", second_data, "mi", 3);

            }


       }
            } else if(test_flag == 1){ //Tasks to be executed while in test mode
                OLEDStringDraw ("Testing Mode  ", 0, 0);
                if (stepcount_flag == 1) {
                    OLEDStringDraw("          ", 0, 2);
                              data = step_counter;
                              data_name = "Steps: ";
                              displayUpdate (data_name, data, 2);
                              OLEDStringDraw("            ", 0, 3);


                        } if(distance_flag == 1) { //Distance Display
                            OLEDStringDraw("Distance:  ", 0, 2);
                            if(kilometre_flag == 1) { //Kilometres
                                data = kilometres;
                                second_data = metres;
                                displayUpdate2 (data, ".", second_data, "km", 3);

                            }  else if (mile_flag == 1) { //Miles
                                data = miles;
                                second_data = milimiles;
                                displayUpdate2 (data, ".", second_data, "mi", 3);

                            }




            }
            }
        i = 0;
      }
      i++;
    }
}
