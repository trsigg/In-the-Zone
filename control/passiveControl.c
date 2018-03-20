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
