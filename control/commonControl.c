#include "..\config\config.c"

#ifdef PASSIVE
	#include "..\control\passiveControl.c"
#else
	#include "..\control\rollerControl.c"
#endif

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>=liftPos[L_SAFE] || abs(goalPower)<=GOAL_STILL_SPEED) {
		setPower(goalIntake, goalPower);
		updateMotorConfig(goalPower);
	}
	else {
		moveLiftToSafePos(false);
	}
}

void handleConeCountInput() {	//change cone count based on user input
	//adjust cone count
	if (newlyPressed(resetBtn))
		numCones = 0;
	else if (numCones<=MAX_NUM_CONES && newlyPressed(increaseConesBtn))
		numCones++;
	else if (numCones>0 && newlyPressed(decreaseConesBtn))
		numCones--;

		if (!bSoundActive && vexRT[sayConeNumberBtn]==1)
			speakNum(numCones);
}

void handleModeInput(bool shift) {
	if (shift && newlyPressed(toggleFieldingBtn)) {
		fielding = !fielding;

		if (fielding)
			playSound(soundDownwardTones);
		else
			playSound(soundUpwardTones);
	}
}

void handleAbortInput() {
	if (newlyPressed(abortManeuversBtn)) {
		stacking = false;
		startAutoStacking();
		stopLiftTargeting();

		#ifdef PASSIVE
			movingToMax = false;
		#endif
	}
}

task usercontrol() {
	stopLiftTargeting();
	setToStillSpeed(roller);
	if (SKILLZ_MODE) moveLiftToSafePos(false);

	startAutoStacking();

	bool shift;

	while (true) {
		logSensorVals();
		updateMotorConfig();
		shift = vexRT[shiftBtn]==1;

		if (shift) handleConeCountInput();

		handleModeInput(shift);
		handleAbortInput();

		handleLiftInput(shift);
		handleGoalIntakeInput();

		#ifndef PASSIVE
			if (!stacking) takeInput(roller);
		#endif

		driveRuntime(drive);

		EndTimeSlice();
	}
}
