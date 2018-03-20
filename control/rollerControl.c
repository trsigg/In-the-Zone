#include "..\config\config.c"

void handleFbInput() {
	if (newlyPressed(toggleFbBtn[robot])) {
		moveFourBar(!fbUp);
	}
	else {
		int fbPower = takeInput(fourBar, fourBar.moving==NO);

		if (fabs(fbPower) > fbStillSpeed[robot])	//update fbUp
			fbUp = fbPower < 0;
	}
}

void handleRollerInput(bool shift) {
	if (!stacking)
		if (AUTOSTACK_CONFIG)
			if (vexRT[roller.posInput] == 1)
				setPower(roller, (shift ? -1 : 1) * 127);
			else
				setPower(roller, rollerStillSpeed[robot]);
		else
			takeInput(roller);
}
