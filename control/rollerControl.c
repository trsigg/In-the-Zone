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
