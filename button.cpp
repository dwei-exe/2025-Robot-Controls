#include "button.h"


void toggleButton(struct toggleButtonObj *button, char newPush, void (*func)(char)) {
	if (!(button->previousPush) && newPush && button->state) {
		button->state = 0;
		(*func)(0);



	} else if (!(button->previousPush) && newPush && !(button->state)) {
		button->state = 1;
		(*func)(1);


	};
	button->previousPush = newPush;

}

