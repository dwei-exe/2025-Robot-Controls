#include "drive25.h"
#include <CrcLib.h>


void calibrateStick(struct Controller *stick) {
    if (stick->x == 1) {
        stick->x = 0;
    }
    if (stick->y == 1) {
        stick->y = 0;
    }
}

