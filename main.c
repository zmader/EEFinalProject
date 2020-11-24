/*
 *  Author: Jordan Jhaveri
 *  Date: 11/17/20
 *
 *  Draws the target for game
 *  the target will eventually have functionality
 *  to destroy asteroids
 *
 */

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#define RAND_MAX 100
#include <stdlib.h>

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer[2];

/*
 * Asteroid struct;
 * contains color (string), coordinates, radius.
 * When the asteroids are created in main,
 * rand() will be used to generate values for
 * x and y.
 *
 */
typedef struct asteroid {
    char* color;
    int Astx;
    int Asty;
    int radius;
};

/*
 * Helper Function to initialize everything
 * It is too long and annoying to look at in main
 */
void initialize(void)
{
    /* Halting WDT and disabling master interrupts */
        MAP_WDT_A_holdTimer();
        MAP_Interrupt_disableMaster();

        /* Set the core voltage level to VCORE1 */
        MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

        /* Set 2 flash wait states for Flash bank 0 and 1*/
        MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
        MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

        /* Initializes Clock System */
        MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
        MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
        MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
        MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
        MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

        /* Initializes display */
        Crystalfontz128x128_Init();
        Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
        Graphics_clearDisplay(&g_sContext);

        /* Set default screen orientation */
        Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

        /* Configures Pin 6.0 and 4.4 as ADC input */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

        /* Initializing ADC (ADCOSC/64/8) */
        MAP_ADC14_enableModule();
        MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

        /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
             * with internal 2.5v reference */
        MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
        MAP_ADC14_configureConversionMemory(ADC_MEM0,
                ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

        MAP_ADC14_configureConversionMemory(ADC_MEM1,
                ADC_VREFPOS_AVCC_VREFNEG_VSS,
                ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

        /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
         *  is complete and enabling conversions */
        MAP_ADC14_enableInterrupt(ADC_INT1);

        /* Enabling Interrupts */
        MAP_Interrupt_enableInterrupt(INT_ADC14);
        MAP_Interrupt_enableMaster();

        /* Setting up the sample timer to automatically step through the sequence
         * convert.
         */
        MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

        /* Triggering the start of the sample */
        MAP_ADC14_enableConversion();
        MAP_ADC14_toggleConversionTrigger();

}
/*
 * Main function
 *
 */
int main(void)
{
    initialize();

    // Initiate push button
    P5->SEL0 &= ~0x02;
    P5->SEL1 &= ~0x02;
    P5->DIR &= ~0x02;
    P5->REN |= 0x02;
    P5OUT |= 0x02;

    int asteroidMax = 20; //maximum # of asteroids on screen at a time
    int numAsteroids = 0; //# of asteroids on screen currently, init. 0

    if(numAsteroids < asteroidMax) {
        GenerateAsteroids("GRAPHICS_COLOR_GRAY", rand(), rand(), 5);
    }

    //Set Background to black, foreground to yellow
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_clearDisplay(&g_sContext);

    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}

void makeTarget(void)
{
    /* Store ADC14 conversion results */
    resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
    resultsBuffer[1] = ADC14_getResult(ADC_MEM1);

    //Draw Circle at joystick coords, scale to screen by dividing by 128
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
    Graphics_drawCircle(&g_sContext, resultsBuffer[0]/128, 128 - resultsBuffer[1]/128, 15);

    //Erase the circle
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_drawCircle(&g_sContext, resultsBuffer[0]/128, 128 - resultsBuffer[1]/128, 15);
}

/*
 * This function should create the asteroids using the values
 * passed from the main
 */
asteroid GenerateAsteroids()

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        makeTarget();
    }
}
