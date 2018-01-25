#include "audioControl.c"
#include "lift.c"


bool stacking = false;	//whether the robot is currently in the process of stacking

//#region sonar autostacking
task sonarAutoStacking() {
	while (!stacking) EndTimeSlice();

	//lift above stack
	setPower(lift, 127);

	bool abort = false;
	while (SensorValue[CONE_SONAR] < CONE_SONAR_THRESH && !abort) {
		if (getPosition(lift) > liftPos[L_MAX]-75)
			abort = true;
		EndTimeSlice();
	}

	moveFourBar(true);

	if (!abort) wait1Msec(50);
	int finishPos = getPosition(lift);

	setLiftTargetAndPID(finishPos);
	waitForMovementToFinish(fourBar);

	//lift down
	//setLiftTargetAndPID(calcLiftTargetForHeight(calcLiftHeight(finishPos) - L_OFFSET));
	setPower(lift, -40);
	wait1Msec(100);

	//outtake
	setPower(roller, -127);
	setLiftTargetAndPID(finishPos);
	waitForMovementToFinish(lift);

	//lift down
	moveFourBar(false, false);
	setLiftState(L_DEF);
}
//#endregion

//#region kinematic autostacking
int numCones = 0; //current number of stacked cones
int liftTarget, liftRelease;

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

task kinematicAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		setLiftTargetAndPID(liftTarget);	//TODO: only target if lift below target
		if (FB_SENSOR >= 0) setFbState(FB_SAFE);
		while (getPosition(lift) < liftRelease) EndTimeSlice();	//wait for lift to move to stacking position

		if (FB_SENSOR >= 0)
			setFbState(STACK);
		else
			moveFourBar(true);

		waitForLiftingToFinish();	//waitForMovementToFinish();

		if (numCones>=MAX_NUM_CONES-1 && HOLD_LAST_CONE) {
			stopAutomovement(lift);	//passively maintains lift position
			lift.stillSpeedReversed = true;
		}
		else {
			stopAutomovement(lift);	//TODO: replace with maneuver?
			setPower(lift, -127);

			wait1Msec(250); //while (getPosition(lift) > liftRelease) EndTimeSlice();
			if (FB_SENSOR >= 0)
				setFbState(FB_DEF);
			else
				moveFourBar(false);

			waitForMovementToFinish(fourBar);
			setLiftState(L_DEF);
		}

		stacking = false;
		numCones++;
		speakNum(numCones);

		/*if (FB_SENSOR >= 0)
			setFbState(FB_DEF);
		else
			moveFourBar(true);*/
	}
}
//#endregion

void startAutoStacking() {
	stacking = false;

	if (SONAR_STACKING && CONE_SONAR>=dgtl1) {
		startTask(sonarAutoStacking);
	}
	else {
		numCones = 0;
		startTask(kinematicAutoStacking);
	}
}

void stackNewCone(bool wait=false) {	//TODO: liftTarget and liftRelease
	if (!(SONAR_STACKING && CONE_SONAR>=dgtl1)) {
		float stackHeight = CONE_HEIGHT * adjustedNumCones();

		liftTarget = calcLiftTargetForHeight(stackHeight + L_OFFSET);	//TODO: noOffsetCones, etc?
		liftRelease = calcLiftTargetForHeight(stackHeight);
	}

	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}
