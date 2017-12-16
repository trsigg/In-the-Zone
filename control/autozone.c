//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"


#ifndef RUN_AUTON_AS_MAIN
	#include "Vex_Competition_Includes.c"
#endif
//#endregion

//#region autopositioning
bool movingToMax = false;	//true if lifting up to MAX_POS - TODO: fb down after going to def?
//#endregion

void pre_auton() {
	#ifndef RUN_AUTON_AS_MAIN
		bStopTasksBetweenModes = true;
	#endif

	initializeStructs();

	initializeAutoMovement();
	driveDefaults.debugStartCol = debugParameters[4];
	turnDefaults.debugStartCol = debugParameters[5];

	if (HAS_SPEAKER)
		initializeAudio();
}

#ifdef RUN_AUTON_AS_MAIN
task main() {
#else
task autonomous() {
#endif
	prepareForAuton();
	handleTesting();
	startTask(autonUpdateTask);

	int sidePos = SensorValue[SIDE_POT];
	int modePos = SensorValue[MODE_POT];

	turnDefaults.reversed = sidePos < 1820;	//TODO: put this val in config

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else if (1270<sidePos && sidePos<2400) {	//passive (does nothing)
		//...
	}
	else if (modePos < 3490) {	//side goal
		sideGoal(modePos<375, false, false, false);

		if (375<modePos && modePos<1960) {	//drive back through cones
			turnDriveTurn(90, 10, 90);
			driveStraight(60);
		}
	}
	else {	//defensive
		driveForDuration(5000, -127);
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

	if (getPosition(lift)>liftPos[L_SAFE] || abs(goalPower)<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
	else
		maybeLiftToSafePos(false);
}

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
	moveLiftToSafePos(false);

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
