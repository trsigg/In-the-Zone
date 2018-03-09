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
			
			fbUp = !takeInput(fourBar);
		}
	}

	executeManeuvers();
}
