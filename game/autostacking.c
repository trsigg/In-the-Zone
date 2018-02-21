#include "audioControl.c"
#include "lift.c"


bool stacking = false;	//whether the robot is currently in the process of stacking
bool goToSafe = false;	//whether lift & fb should go to safe pos (as opposed to def pos) after stacking

//#region sonar autostacking
//TODO: fix cone drops

task sonarAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		//lift above stack
		bool abort = liftUntilSonar(true);

		moveFourBar(true, false);

		/*if (!abort) wait1Msec(50);
		int finishPos = getPosition(lift);

		setLiftTargetAndPID(finishPos);
		waitForMovementToFinish(fourBar);*/

		//lift down
		//setLiftTargetAndPID(calcLiftTargetForHeight(calcLiftHeight(finishPos) - L_OFFSET));
		liftUntilSonar(false);

		//outtake
		setPower(roller, -127);
		liftUntilSonar(true);
		/*setLiftTargetAndPID(finishPos);
		waitForMovementToFinish(lift);*/

		//lift down
		moveFourBar(goToSafe, false);
		setToStillSpeed(roller);
		setLiftState(goToSafe ? L_SAFE : L_DEF);

		stacking = false;
	}
}
//#endregion

//#region kinematic autostacking
int liftTarget, liftRelease;

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

task kinematicAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		setLiftTargetAndPID(liftTarget);	//TODO: if (getPosition(lift) < liftTarget)?
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
			#ifdef PASSIVE
				stopAutomovement(lift);	//TODO: replace with maneuver?
				setPower(lift, -127);

				wait1Msec(250); //while (getPosition(lift) > liftRelease) EndTimeSlice();
			#else
				/*stopAutomovement(lift);	//TODO: replace with maneuver?
				setPower(lift, -127);

				long stackTimer = resetTimer();
				while (getPosition(lift) > liftRelease && time(stackTimer) < 150) EndTimeSlice();*/
				setLiftTargetAndPID(liftRelease);
				waitForMovementToFinish(lift);

				setPower(roller, -127);
				setLiftTargetAndPID(liftTarget);
				waitForMovementToFinish(lift);
				setToStillSpeed(roller);
			#endif

			if (FB_SENSOR >= 0)
				setFbState(goToSafe ? FB_SAFE : FB_DEF);
			else
				moveFourBar(goToSafe);

			waitForMovementToFinish(fourBar);
			setLiftState(goToSafe ? L_SAFE : L_DEF);
			setToStillSpeed(roller);
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

	if (CONE_SONAR>=dgtl1 && SONAR_STACKING && !bIfiAutonomousMode)
		startTask(sonarAutoStacking);
	else
		startTask(kinematicAutoStacking);
}

void stackNewCone(bool waite=false, bool safeAtEnd=false) {	//TODO: liftTarget and liftRelease
	if (!(SONAR_STACKING && CONE_SONAR>=dgtl1)) {
		float stackHeight = CONE_HEIGHT * adjustedNumCones();

		liftTarget = calcLiftTargetForHeight(stackHeight + L_OFFSET);	//TODO: noOffsetCones, etc?
		liftRelease = calcLiftTargetForHeight(stackHeight);
	}

	goToSafe = safeAtEnd;
	stacking = true;

	if (waite)
		while (stacking)
			EndTimeSlice();
}
