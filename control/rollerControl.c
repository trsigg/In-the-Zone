void handleAutopositioningInput(bool shift) {
	if (!shift)
		if (newlyPressed(defPosBtn))
			setLiftState(L_DEF);

		if (newlyPressed(toggleFbBtn))
			moveFourBar(!fbUp);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift && vexRT[stackBtn]==1) {
			stackNewCone();
		}
		else {
			handleAutopositioningInput(shift);

			//will only set power if not maintaining a position
			//if there is input, activelyMaintaining will be set to false and normal control will resume
			takeInput(lift, lift.moving==NO && !stacking);
			int fbPower = takeInput(fourBar, fourBar.moving==NO);

			if (fabs(fbPower) > FB_STILL_SPEED)	//update fbUp
				fbUp = fbPower < 0;
		}
	}

	executeManeuvers();
}
