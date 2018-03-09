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


bool movingToMax = false;


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

void handleAutopositioningInput(bool shift) {
	if (!shift) {
		if (safePosBtn[robot]>=0 && newlyPressed(safePosBtn[robot])) {
			setLiftState(L_SAFE);
			movingToMax = false;
		}

		if (maxPosBtn[robot]>=0 && newlyPressed(maxPosBtn[robot])) {
			setLiftState(SKILLZ_MODE ? L_SAFE : L_MAX);
			movingToMax = true;
		}
	}

	if (movingToMax && errorLessThan(lift, 100)) {
		moveFourBar(true);

		movingToMax = false;
	}
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
		}
	}

	executeManeuvers();
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
		handleFbInput();
		handleGoalIntakeInput();

		#ifdef ROLLER
			handleRollerInput();
		#endif

		#ifdef PNEUMATIC
			takeInput(intake);
		#endif

		driveRuntime(drive);

		EndTimeSlice();
	}
}
