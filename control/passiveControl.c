void handleFbInput() {
	int fbPower = takeInput(fourBar, fourBar.moving==NO);

	if (fabs(fbPower) > fbStillSpeed[robot])	//update fbUp
		fbUp = fbPower < 0;
}
