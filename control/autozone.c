//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"	//TODO: main testing
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"
#include "..\game\testing.c"
//#endregion

//#region autopositioning
bool movingToMax = false;	//true if lifting up to MAX_POS - TODO: fb down after going to def?
//#endregion

void pre_auton() {
	bStopTasksBetweenModes = true;	//TODO: main testing

	initializeStructs();

	initializeAutoMovement();
	driveDefaults.debugStartCol = debugParameters[4];
	turnDefaults.debugStartCol = debugParameters[5];

	if (HAS_SPEAKER)
		initializeAudio();
}

task autonomous() {	//TODO: main testing
	prepareForAuton();
	handleTesting();

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else {
		turnDefaults.reversed = SensorValue[SIDE_POT]<1830;	//TODO: put this val in config
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

void handleAutopositioningInput(bool shift) {
	if (!shift) {
		if (newlyPressed(defPosBtn)) {
			setLiftState(L_DEF);
			movingToMax = false;
		}

		if (newlyPressed(maxPosBtn)) {
			setLiftState(L_MAX);
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

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>liftPos[L_SAFE] || goalPower<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (vexRT[stackBtn] == 1) {
			movingToMax = false;
			stackNewCone();
		}
		else {
			handleAutopositioningInput(shift);

			takeInput(fourBar, fourBar.moving==NO); //will only set power if not maintaining a position
			takeInput(lift, lift.moving==NO);       //if there is input, activelyMaintaining will be set to false and normal control will resume
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
			movingToMax = false;
			startTask(autoStacking);
			stopLiftTargeting();
		}

		handleLiftInput(shift);
		handleGoalIntakeInput();

		driveRuntime(drive);
	}
}
//#endregion
