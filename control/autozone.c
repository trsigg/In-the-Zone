//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"
#include "..\game\testing.c"
//#endregion

//#region autopositioning
enum AutoPosState { NO_POS, CHAIN_DEF, CH_STACK, FULL_DEF, MAX };
AutoPosState posState = NO_POS;
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeStructs();
	initializeAutoMovement();
	if (HAS_SPEAKER)
		initializeAudio();

	setLiftControlMode(fielding);
}

task autonomous() {
	prepareForAuton();
	handleTesting();

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else {
		turnDefaults.reversed = SensorValue[SIDE_POT]<1830;
		startTask(sideGoalTask);
	}

	while (true) {
		executeLiftManeuvers();
		logSensorVals();
		EndTimeSlice();
	}
}

//#region usercontrol
void adjustConeCount() {	//change cone count based on user input
	if (newlyPressed(resetBtn))
		numCones = 0;

	if (numCones<=MAX_NUM_CONES && newlyPressed(increaseConesBtn))
		numCones++;

	if (numCones>0 && newlyPressed(decreaseConesBtn))
		numCones--;
}

void setAutopositionState(AutoPosState state) {
	stacking = false;	//TODO: ?
	posState = state;
	startTask(autoStacking);	//reset task progress

	switch (state) {
		case NO_POS:
			stopLiftTargeting();
			break;
		case CHAIN_DEF:
			setChainBarState(CH_DEF);
			break;
		case CH_STACK:
			setChainBarState(STACK);
			break;
		case FULL_DEF:
			setChainBarState(CH_DEF);
			setLiftState(L_DEF);
			break;
		case MAX:
			setLiftState(L_MAX);
			break;
	}
}

void handleAutopositioningInput() {
	if (vexRT[chainDefBtn] == 1) {
		if (vexRT[chainStackBtn]==1 && posState!=FULL_DEF)
			setAutopositionState(FULL_DEF);
		else if (posState!=CHAIN_DEF && posState!=FULL_DEF)
			setAutopositionState(CHAIN_DEF);
	}
	else if (vexRT[chainStackBtn] == 1) {
		if (posState!=CH_STACK && posState!=FULL_DEF)
			setAutopositionState(CH_STACK);
	}
	/*else if (vexRT[maxPosBtn] == 1) {
		if (posState != MAX)
			setAutopositionState(MAX);
	}*/
	else {
		posState = NO_POS;
	}
}

void handleConeIntakeInput(bool shift) {
	if (vexRT[coneIntakeBtn] == 1)
		if (shift)
			setPower(coneIntake, -127);
		else
			setPower(coneIntake, 127);
	else
		setPower(coneIntake, INTAKE_STILL_SPEED);
}

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>liftPos[L_SAFE] || goalPower<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift) {
			if (vexRT[stackBtn] == 1) {
				stackNewCone();
			}
			else {
				handleAutopositioningInput();
				takeInput(chainBar, !chainBar.activelyMaintining); //will only set power if not maintaining a position
			  takeInput(lift, !lift.activelyMaintining);         //if there is input, activelyMaintaining will be set to false and normal control will resume
			}
		}

		handleConeIntakeInput(shift);
	}

	executeLiftManeuvers();
}

task usercontrol() {
	stopLiftTargeting();

	startTask(autoStacking);

	bool shift;

	while (true) {
		logSensorVals();
		shift = vexRT[shiftBtn]==1;

		if (shift) {
			adjustConeCount();

			if (newlyPressed(resetEncodersBtn))
				resetLiftEncoders();

			if (!bSoundActive && vexRT[sayConeNumberBtn]==1)
				speakNum(numCones);

			if (newlyPressed(toggleFieldingBtn)) {
				fielding = !fielding;

				if (fielding)
					playSound(soundDownwardTones);
				else
					playSound(soundUpwardTones);

				setLiftControlMode(fielding);
			}
		}

		if (newlyPressed(abortManeuversBtn)) {
			stacking = false;
			startTask(autoStacking);
			stopLiftTargeting();
		}

		handleLiftInput(shift);
		handleGoalIntakeInput();

		driveRuntime(drive);
	}
}
//#endregion
