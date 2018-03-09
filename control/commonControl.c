#include "..\config\config.c"

#ifdef PASSIVE
	#include "..\control\passiveControl.c"
#endif

#ifdef ROLLER
	#include "..\control\rollerControl.c"
#endif

#ifdef PNEUMATIC
	#include "..\control\pneumaticControl.c"
#endif

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>=liftPos[L_SAFE] || abs(goalPower)<=goalStillSpeed[robot]) {
		setPower(goalIntake, goalPower);
		updateMotorConfig(goalPower);
	}
	else {
		moveLiftToSafePos(false);
	}
}

void handleConeCountInput() {	//change cone count based on user input
	//adjust cone count
	if (newlyPressed(resetConesBtn))
		numCones = 0;
	else if (numCones<=maxNumCones[robot] && newlyPressed(increaseConesBtn))
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
	bool shift;

	stopLiftTargeting();
	startAutoStacking();

	if (SKILLZ_MODE) moveLiftToSafePos(false);

	#ifdef ROLLER
		setToStillSpeed(roller);
	#endif

	while (true) {
		logSensorVals();
		updateMotorConfig();
		shift = vexRT[shiftBtn]==1;

		if (shift) handleConeCountInput();

		handleModeInput(shift);
		handleAbortInput();

		handleLiftInput(shift);
		handleGoalIntakeInput();

		#ifdef ROLLER
			handleRollerInput();
		#endif

		driveRuntime(drive);

		EndTimeSlice();
	}
}
