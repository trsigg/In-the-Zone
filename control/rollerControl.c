#include "commonControl.c"


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

task usercontrol() {
	stopLiftTargeting();
	if (SKILLZ_MODE) moveLiftToSafePos(false);

	startAutoStacking();

	bool shift;

	while (true) {
		logSensorVals();
		shift = vexRT[shiftBtn]==1;

		if (shift) {
			adjustConeCount();

			if (!bSoundActive && vexRT[sayConeNumberBtn]==1)
				speakNum(numCones);
		}

		if (newlyPressed(toggleFieldingBtn)) {
				fielding = !fielding;

				if (fielding)
					playSound(soundDownwardTones);
				else
					playSound(soundUpwardTones);
			}

		if (newlyPressed(abortManeuversBtn)) {
			stacking = false;
			movingToMax = false;
			startTask(autoStacking);
			stopLiftTargeting();
		}

		handleLiftInput(shift);
		handleGoalIntakeInput();

		driveRuntime(drive);
	}
}
