bool fbIn = false;	//true if lifting up to MAX_POS - TODO: fb down after going to def?

void handleAutopositioningInput(bool shift) {
	if (!shift)
		if (newlyPressed(toggleFbBtn))
			moveFourBar(fbIn = !fbIn);
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
			takeInput(lift, lift.moving==NO);
			int fbPower = takeInput(fourBar, fourBar.moving==NO);

			if (fabs(fbPower) > FB_STILL_SPEED)	//update fbIn
				fbIn = fbPower > 0;	//TODO: reverse?
		}
	}

	executeManeuvers();
}
