/*
 * target.c
 *
 *  Created on: Nov 30, 2020
 *      Author: jhaveri
 */
#include "target.h"

enum xDirection {right = 3, left = -3};
enum yDirection {up = -3, down = 3};

/*
 * Initialize target object
 * take in coordinates where target will first spawn
 */
target init_target(int targetX, int targetY, int radius)
{
    target newTarget;

    newTarget.targX = targetX;
    newTarget.targY = targetY;
    newTarget.targRadius = radius;

    return newTarget;
}

/*
 * checkBounds allows to see if the movement is legal
 * call this function in move target, use enum so I
 * don't have to keep track of the directions being opposite
 * what I think
 */
int checkBounds(target * newTarget, int xDir, int yDir)
{
 if(newTarget->targX + xDir > (128 - newTarget->targRadius) && xDir == right)
 {
     return 0;
 }
 if(newTarget->targX - xDir < newTarget->targRadius && xDir == left)
 {
     return 0;
 }
 if(newTarget->targY - yDir < newTarget->targRadius && yDir == up)
 {
     return 0;
 }
 if(newTarget->targY + yDir > (128 - newTarget->targRadius) && yDir == down)
 {
     return 0;
 }

 return 1;
}

/*
 * Function that creates a target controlled by joystick
 * passes in a pointer to the resultsBuffer to gain access to values
 */
void moveTarget(Graphics_Context *g_sContext, uint16_t *resultsBuffer, target *myTarget)
{
    //Erase the circle
    Graphics_setForegroundColor(g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_drawCircle(g_sContext, myTarget->targX, myTarget->targY, myTarget->targRadius);

    int xDir, yDir;

    if(resultsBuffer[1] > 12000)
    { //move to the up
        yDir = up;
    }
    else if(resultsBuffer[1] < 6000)
    { //move to the down
        yDir = down;
    }
    else
    {
        yDir = 0;
    }
    //determine where the target will start going
    if(resultsBuffer[0] > 12000)
    { //move to the right
        xDir = right;
    }

    else if(resultsBuffer[0] < 6000)
     { //move to the left
         xDir = left;
     }
    else
    {
        xDir = 0;
    }

    if(checkBounds(myTarget, xDir, yDir))
    {
        myTarget->targX += xDir;
        myTarget->targY = myTarget->targY + yDir;
        //Draw Circle at joystick coords, scale to screen by dividing by 128
        Graphics_setForegroundColor(g_sContext, GRAPHICS_COLOR_YELLOW);
        Graphics_drawCircle(g_sContext, myTarget->targX, myTarget->targY, myTarget->targRadius);
    }
}

