//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"
#include "..\game\testing.c"
//#endregion

//#region autopositioning
bool movingToMax = false;	//true if lifting up to MAX_POS
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeStructs();
	initializeAutoMovement();
	if (HAS_SPEAKER)
		initializeAudio();
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

void handleAutopositioningInput() {
	if (newlyPressed(defPosBtn)) {
		setState(coneIntake, true);
		setState(fourBar, false);
		setLiftState(L_DEF);
	}

	if (newlyPressed(maxPosBtn)) {
		setLiftState(L_MAX);
		movingToMax = true;
	}

	if (movingToMax && errorLessThan(lift, 100)) {
		setState(fourBar, true);
		movingToMax = false;
	}
}

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>liftPos[L_SAFE] || goalPower<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
}

void handleLiftInput() {
	if (!stacking) {
		if (vexRT[stackBtn] == 1) {
			stackNewCone();
		}
		else {
			handleAutopositioningInput();
			takeInput(lift, !lift.activelyMaintining); //will only set power if not maintaining a position
		                                             //if there is input, activelyMaintaining will be set to false and normal control will resume
		}
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
			startTask(autoStacking);
			stopLiftTargeting();
		}

		handleLiftInput();

		takeInput(coneIntake);
		takeInput(fourBar);

		driveRuntime(drive);
	}
}
//#endregion
