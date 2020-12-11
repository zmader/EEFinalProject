/*
 * asteroid.h
 *
 *  Created on: Nov 30, 2020
 *      Author: jhave
 */

#ifndef ASTEROID_H_
#define ASTEROID_H_

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define asteroidMax 5
/*
 * Asteroid struct;
 * contains color (string), coordinates, radius.
 * When the asteroids are created in main,
 * rand() will be used to generate values for
 * x and y.
 *
 */
typedef struct asteroid {
    int Astx;
    int Asty;
    int radius;
    int xVel;
    int yVel;
}asteroid;

void initButton(void);
void initJoystick(void);
asteroid* initAsteroid(int maxAsteroids, int asteroidR);
void drawAsteroids(Graphics_Context *g_sContext, asteroid* array);
void Init_Graph(Graphics_Context* g_sContext_f);
int randNum(void);
void ADC14_IRQHandler(void);
int checkIfDestroyed(Graphics_Context *g_sContext, asteroid* array, int cursorX, int cursorY); //added 12/1/2020
void moveAsteroids(Graphics_Context *g_sContext, asteroid* array); //added 12/1/2020


#endif /* ASTEROID_H_ */
