/*
 * asteroid.c
 * Initializes the asteroids.
 *
 *  Created on: Nov 30, 2020
 *      Author: jhaveri
 */

#include "asteroid.h"

/*
 * Initialize push button
 */
void initButton(void)
{
    // Initialize push button
    P5->SEL0 &= ~0x02;
    P5->SEL1 &= ~0x02;
    P5->DIR &= ~0x02;
    P5->REN |= 0x02;
    P5OUT |= 0x02;
}

int randNum(void)
{
    //srand(time(0));
    //mod with 115 so it doesn't go out of bounds
    return (10 + (rand() % (100))); //offset it so that it doesnt get stuck in the corners
}

asteroid* initAsteroid(int maxAsteroids, int asteroidR)
{
    struct asteroid *asteroidArray = (struct asteroid*)malloc(sizeof(struct asteroid)*maxAsteroids);

    int i = 0;
    while(maxAsteroids > 0)
       {
        asteroidArray[i].Astx = randNum();
        asteroidArray[i].Asty = randNum();
        asteroidArray[i].radius = asteroidR;
        asteroidArray[i].xVel = (rand() % 6) - 3;
        asteroidArray[i].yVel = (rand() % 6) - 3;
        maxAsteroids--;
        i++;
       }

    return asteroidArray;
}

void drawAsteroids(Graphics_Context *g_sContext, asteroid* array)
{
    Graphics_setForegroundColor(g_sContext, GRAPHICS_COLOR_WHITE);

    int i;
    for(i = 0; i < asteroidMax; i++)
    {
        Graphics_fillCircle(g_sContext, array[i].Astx, array[i].Asty, array[i].radius);

    }
}

int checkIfDestroyed(Graphics_Context *g_sContext, asteroid* array, int cursorX, int cursorY)
{
    //check if the button is pressed, if not, return


    int targetRange = 7; //for now the range is 20
    int asteroidsDestroyed = 0; //to return how many asteroids were removed on this call

    //for each element in the asteroid array
    int i;
    for(i = 0; i < asteroidMax; i++)
    {
        //check if the center of the asteroid is within a reasonable range of the center of the target
        if(cursorX < array[i].Astx + targetRange && cursorX > array[i].Astx - targetRange) //check the x coordinates first
        {
            if(cursorY < array[i].Asty + targetRange && cursorY > array[i].Asty - targetRange) //check the y coordinates first
            {
                //draw over the asteroid so it disappears
                Graphics_setForegroundColor(g_sContext, GRAPHICS_COLOR_BLACK);
                Graphics_fillCircle(g_sContext, array[i].Astx, array[i].Asty, array[i].radius);
                //remove the asteroid (for testing, it just draws it at the center for now)
                array[i].Astx = randNum();
                array[i].Asty = randNum();
                asteroidsDestroyed += 1;
            }
        }
    }
    //maybe keep a count of how many asteroids were destroyed on each go, and total count is held in main?
    //we could also reuse the already allocated spots by randomly setting the position again (randNum())
    return asteroidsDestroyed;
}

void moveAsteroids(Graphics_Context *g_sContext, asteroid* array)
{
    Graphics_setForegroundColor(g_sContext, GRAPHICS_COLOR_BLACK);
    int i;
    for(i = 0; i < asteroidMax; i++)
    {
        Graphics_fillCircle(g_sContext, array[i].Astx, array[i].Asty, array[i].radius);
        array[i].Astx += array[i].xVel;
        array[i].Asty += array[i].yVel;
        if(array[i].Astx < 0)
        {
            array[i].Astx = LCD_HORIZONTAL_MAX;
        }
        else if(array[i].Astx > LCD_HORIZONTAL_MAX)
        {
            array[i].Astx = 0;
        }
        if(array[i].Asty < 0)
        {
            array[i].Asty = LCD_VERTICAL_MAX;
        }
        else if(array[i].Asty > LCD_VERTICAL_MAX)
        {
            array[i].Asty = 0;
        }
    }
    drawAsteroids(g_sContext, array);
}


