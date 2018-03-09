void handleFbInput() {
	int fbPower = takeInput(fourBar, fourBar.moving==NO);

	if (fabs(fbPower) > fbStillSpeed[robot])	//update fbUp
		fbUp = fbPower < 0;
}

void handleRollerInput() {
	if (!stacking)
		if (AUTOSTACK_CONFIG)
			if (vexRT[roller.posInput] == 1)
				setPower(roller, (shift ? -1 : 1) * 127);
			else
				setPower(roller, rollerStillSpeed[robot]);
		else
			takeInput(roller);
}
