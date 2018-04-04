#include "audioControl.c"
#include "lift.c"


bool stacking = false;	//whether the robot is currently in the process of stacking
bool goToSafe = false;	//whether lift & fb should go to safe pos (as opposed to def pos) after stacking
void startAutoStacking(bool sonar = (coneSonar[robot]>=dgtl1 && SONAR_STACKING && !(bIfiAutonomousMode || SKILLZ_MODE)));

void setIntakeState(bool intakeCone) {
	#ifdef ROLLER
		setPower(roller, (intakeCone ? 1 : -1)*127);
	#endif

	#ifdef PNEUMATIC
		setState(intake, intakeCone);
	#endif
}

void liftDown() {
	moveFourBar(goToSafe, false);

	#ifdef ROLLER
		setToStillSpeed(roller);
		//while (getPosition(fourBar) < fbPos[FB_SAFE]) EndTimeSlice();
	#endif

	#ifdef PNEUMATIC
		setState(intake, true);
	#endif

	if (goToSafe)
		liftToConeSafePos();
	else
		setLiftState(L_DEF, true);
}

//#region sonar autostacking
//TODO: fix cone drops

task sonarAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		if (SKILLZ_MODE) {
			setIntakeState(true);
			moveFourBar(false, false);
			while (getPosition(fourBar) < fbPos[FB_SAFE]) EndTimeSlice();
			setLiftState(L_DEF, true);
			waitForMovementToFinish(lift);
			//wait1Msec(intakeDuration[robot]);

			#ifdef ROLLER
				setToStillSpeed(roller);
			#endif
		}

		//lift above stack
		liftUntilSonar(true);

		moveFourBar(true, false);

		//lift down
		//liftUntilSonar(false);
		moveForDuration(lift, -127, 150, false);

		//outtake
		setIntakeState(false);
		//moveForDuration(lift, 127, 250, false);
		liftUntilSonar(true, 75);	//TODO: or above where it dropped cone off (use dangerPos mechanism?)

		//lift down
		liftDown();

		stacking = false;
	}
}
//#endregion

//#region kinematic autostacking
int liftTarget, liftRelease;

int adjustedNumCones() {
	return limit(numCones-apathyCones[robot], 0, maxNumCones[robot]-apathyCones[robot]);
}

task kinematicAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		setLiftTargetAndPID(liftTarget);	//TODO: if (getPosition(lift) < liftTarget)?
		while (getPosition(lift) < liftRelease) EndTimeSlice();	//wait for lift to move to stacking position

		moveFourBar(true);

		waitForLiftingToFinish();	//waitForMovementToFinish();

		if (numCones>=maxNumCones[robot]-1 && HOLD_LAST_CONE) {
			stopAutomovement(lift);	//passively maintains lift position
			lift.stillSpeedReversed = true;

			#ifdef PASSIVE
				setState(intake, false);
			#endif
		}
		else {
			#ifdef PASSIVE
				stopAutomovement(lift);	//TODO: replace with maneuver?
				setPower(lift, -127);

				wait1Msec(outtakeDuration[robot]); //while (getPosition(lift) > liftRelease) EndTimeSlice();

				/*stopAutomovement(lift);	//TODO: replace with maneuver?
				setPower(lift, -127);

				long stackTimer = resetTimer();
				while (getPosition(lift) > liftRelease && time(stackTimer) < 150) EndTimeSlice();*/
			#else
				setLiftTargetAndPID(liftRelease);
				waitForMovementToFinish(lift);

				setIntakeState(false);
				setLiftTargetAndPID(liftTarget);
				waitForMovementToFinish(lift, outtakeDuration[robot]);
			#endif

			liftDown();
		}

		stacking = false;
		numCones++;
		speakNum(numCones);

		if (SKILLZ_MODE && SONAR_IN_SKILLZ) break;
	}

	startAutoStacking(true);
}
//#endregion

void startAutoStacking(bool sonar) {	//prototype with default above
	stacking = false;

	if (sonar)
		startTask(sonarAutoStacking);
	else
		startTask(kinematicAutoStacking);
}

void stackNewCone(bool waite=false, bool safeAtEnd=bIfiAutonomousMode||SKILLZ_MODE) {	//TODO: liftTarget and liftRelease
	if (!(SONAR_STACKING && coneSonar[robot]>=dgtl1) || bIfiAutonomousMode) {
		float stackHeight = coneHeight[robot] * adjustedNumCones();

		liftTarget = calcLiftTargetForHeight(stackHeight + l_offset[robot]);	//TODO: noOffsetCones, etc?
		liftRelease = calcLiftTargetForHeight(stackHeight);
	}

	goToSafe = safeAtEnd;
	stacking = true;

	if (waite)
		while (stacking)
			EndTimeSlice();
}
