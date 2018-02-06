bool movingToMax = false;	//true if lifting up to MAX_POS - TODO: fb down after going to def?

void handleAutopositioningInput(bool shift) {
	if (!shift) {
		if (newlyPressed(defPosBtn)) {
			setLiftState(L_DEF);
			movingToMax = false;
		}

		if (newlyPressed(maxPosBtn)) {
			setLiftState(SKILLZ_MODE ? L_SAFE : L_MAX);
			movingToMax = true;
		}
	}

	if (movingToMax && errorLessThan(lift, 100)) {
		if (FB_SENSOR >= 0)
			setFbState(STACK);
		else
			moveFourBar(true);

		movingToMax = false;
	}
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift && vexRT[stackBtn]==1) {
			movingToMax = false;
			stackNewCone();
		}
		else {
			handleAutopositioningInput(shift);

			takeInput(fourBar, fourBar.moving==NO); //will only set power if not maintaining a position
			takeInput(lift, lift.moving==NO);       //if there is input, activelyMaintaining will be set to false and normal control will resume
		}
	}

	executeManeuvers();
}
