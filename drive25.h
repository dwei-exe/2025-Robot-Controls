#ifndef DRIVE_H
#define DRIVE_H

struct Controller {
    short int x;
    short int y;
};

typedef Controller Vector;

void calibrateStick(struct Controller *stick);

void stopMouvement();


#endif
