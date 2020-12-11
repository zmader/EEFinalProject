//John Costales
//Jordan Jhaveri
//Zack Mader

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include "LcdDriver/HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "target.h"
#include "asteroid.h"
void titleScreen(Graphics_Context* g_sContext);

/* On the off chance we make it past level one
 * and want to have different colored asteroids
 * that need different number of hits.
 */
#define ASTEROID_LEVEL1 GRAPHICS_COLOR_GRAY
#define BACKGROUND GRAPHICS_COLOR_BLACK

#define minCoords 0
#define maxCoords 128
#define asteroidMax 5
#define string_size 20
#define timerCount 20

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer[2];

/* # of asteroids on screen currently, init. 0 */
int numAsteroids = 0;
int asteroidsDestroyed = 0;

/*
 * Main function
 *
 */
int main(void)
{
    Init_Graph(&g_sContext);
    initJoystick();
    initButton();

    // initialize LED
    P2->SEL0 &= ~0x10;
    P2->SEL1 &= ~0x10;
    P2->DIR |= 0x10;
    P2OUT = 0x00;

    titleScreen(&g_sContext);

    asteroid* asteroidArray = initAsteroid(asteroidMax, 5);

    //create target in center of screen w/ radius = 10
    target shooter = init_target(64, 64, 10);

    //to keep track of asteroids destroyed
    char myString[string_size];

    //to keep track of time
    char timerString[string_size];

    //variables for timer
    int counter_value, time_lapsed_s;
    time_lapsed_s = 0;
    int start_counter = timerCount*50000000; // max count 20 seconds

    //start timer
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_haltTimer (TIMER32_0_BASE);
    Timer32_setCount (TIMER32_0_BASE, start_counter);
    Timer32_startTimer(TIMER32_0_BASE, true);

    bool timer = true;

    while(timer)
    {
        counter_value = Timer32_getValue(TIMER32_0_BASE);
        time_lapsed_s = (start_counter-counter_value)/(50000000); //seconds

        //timer
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        snprintf(timerString, string_size, "%02d", (timerCount-time_lapsed_s));
        Graphics_drawString(&g_sContext, (int8_t*)timerString, -1, 5, 120, true);

        //asteroid count
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
        snprintf(myString, 20, "%d Asteroids", asteroidsDestroyed);
        Graphics_drawString(&g_sContext, (int8_t*)myString, -1, 5, 5, true);

        moveTarget(&g_sContext, resultsBuffer, &shooter);
        //drawAsteroids(&g_sContext, asteroidArray);

        //check coordinates of asteroid compared to target
        if(~P5IN & 0x02)
        {
            if (checkIfDestroyed(&g_sContext, asteroidArray, shooter.targX, shooter.targY) != 0)
            {
                P2OUT = 0x10; //turn on green LED if asteroid was destroyed
                asteroidsDestroyed++;
            }
        }

        moveAsteroids(&g_sContext, asteroidArray);
        P2OUT = 0x00;

        if(time_lapsed_s >= 20)
        {
            timer = false;
        }
    }

    Graphics_clearDisplay(&g_sContext);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setFont(&g_sContext, &g_sFontCm22b);
    Graphics_drawString(&g_sContext, "Game Over", -1, 5, 40, true);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    Graphics_setFont(&g_sContext, &g_sFontCm20);
    snprintf(myString, 20, "%d asteroids", asteroidsDestroyed);
    Graphics_drawString(&g_sContext, (int8_t*)myString, -1, 18, 65, true);
}

//Initialize joystick settings
void initJoystick(void)
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

//Initializing the graphics
void Init_Graph(Graphics_Context* g_sContext_f)
{
    // Initialize the LCD
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    // Initialize context
    Graphics_initContext(g_sContext_f, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);

    // Set colors and fonts
    Graphics_setForegroundColor(g_sContext_f, ASTEROID_LEVEL1);
    Graphics_setBackgroundColor(g_sContext_f, BACKGROUND);
    Graphics_setFont(g_sContext_f, &g_sFontFixed6x8);

    // Clear the screen
    Graphics_clearDisplay(g_sContext_f);
}

/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
    }

}

void titleScreen(Graphics_Context* g_sContext)
{
    extern tImage TitleScreen00004BPP_UNCOMP;

    Graphics_drawImage(g_sContext, &TitleScreen00004BPP_UNCOMP, 0, 0);

    while(P5IN & 0x02) {
        //do nothing
    }
    Graphics_clearDisplay(g_sContext);
    return;
}
