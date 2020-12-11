/*
 * target.h
 *
 *  Created on: Nov 30, 2020
 *      Author: jhave
 */
#include <ti/grlib/grlib.h>
#ifndef TARGET_H_
#define TARGET_H_

/*
 * struct to make target
 * needs x and y coordinates
 *
 * needs to be a  struct so we can
 * pass the whole object and reference
 * object attributes, and coordinates
 * will carry over to main.
 */
typedef struct target {
    int targX;
    int targY;
    int targRadius;
}target;

target init_target(int targetX, int targetY, int radius);
int checkBounds(target * newTarget, int xDir, int yDir);
void moveTarget(Graphics_Context *g_sContext, uint16_t *resultsBuffer, target *myTarget);

#endif /* TARGET_H_ */
