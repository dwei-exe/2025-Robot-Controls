#ifndef BUTTON_H
#define BUTTON_H


struct toggleButtonObj { 
	char state;
	char previousPush;
};

void toggleButton(struct toggleButtonObj *button, char newPush, void (*func)(char));

#endif
