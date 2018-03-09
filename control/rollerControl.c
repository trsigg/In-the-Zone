void handleAutopositioningInput(bool shift) {
	if (!shift)
		if (newlyPressed(defPosBtn[robot]))
			setLiftState(L_SAFE/*L_DEF*/);

		if (newlyPressed(toggleFbBtn[robot]))
			moveFourBar(!fbUp);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift && vexRT[stackBtn]==1 && AUTOSTACK_CONFIG) {
			stackNewCone();
		}
		else {
			handleAutopositioningInput(shift);

			//will only set power if not maintaining a position
			//if there is input, activelyMaintaining will be set to false and normal control will resume
			takeInput(lift, lift.moving==NO && !stacking);
			int fbPower = takeInput(fourBar, fourBar.moving==NO);

			if (fabs(fbPower) > fbStillSpeed[robot])	//update fbUp
				fbUp = fbPower < 0;
		}
	}

	executeManeuvers();
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
