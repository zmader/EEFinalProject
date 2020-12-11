/*
 * header.h
 *
 *  Created on: Nov 30, 2020
 *      Author: jhaveri
 */

#ifndef HEADER_H_
#define HEADER_H_
#include <ti/grlib/grlib.h>
#include <stdlib.h>

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
}asteroid;

void initButton(void);
void initJoystick(void);
void Init_Graph(Graphics_Context* g_sContext_f);
void makeTarget(uint16_t *resultsBuffer, int xPos, int yPos);
int randNum(void);
void ADC14_IRQHandler(void);

#endif /* HEADER_H_ */
