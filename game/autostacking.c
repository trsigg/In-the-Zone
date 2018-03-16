#include "audioControl.c"
#include "lift.c"


bool stacking = false;	//whether the robot is currently in the process of stacking
bool goToSafe = false;	//whether lift & fb should go to safe pos (as opposed to def pos) after stacking

void outtake() {
	#ifdef ROLLER
		setPower(roller, -127);
	#endif

	#ifdef PNEUMATIC
		setState(intake, false);
	#endif
}

void prepareToLiftDown() {
	#ifdef ROLLER
		setToStillSpeed(roller);
	#endif

	#ifdef PNEUMATIC
		setState(intake, true);
	#endif
}

//#region sonar autostacking
//TODO: fix cone drops

task sonarAutoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		//lift above stack
		liftUntilSonar(true);

		moveFourBar(true, false);

		//lift down
		//liftUntilSonar(false);
		moveForDuration(lift, -60, 150, false);

		//outtake
		outtake();
		moveForDuration(lift, 127, 250, false);
		//liftUntilSonar(true);

		//lift down
		moveFourBar(goToSafe, false);
		prepareToLiftDown();
		setLiftState(goToSafe ? L_SAFE : L_DEF, true);

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

				outtake();
				setLiftTargetAndPID(liftTarget);
				waitForMovementToFinish(lift);
			#endif

			moveFourBar(goToSafe);
			waitForMovementToFinish(fourBar);

			prepareToLiftDown();
			setLiftState(goToSafe ? L_SAFE : L_DEF, true);
		}

		stacking = false;
		numCones++;
		speakNum(numCones);

		//moveFourBar(true);
	}
}
//#endregion

void startAutoStacking() {
	stacking = false;

	if (coneSonar[robot]>=dgtl1 && SONAR_STACKING && !bIfiAutonomousMode)
		startTask(sonarAutoStacking);
	else
		startTask(kinematicAutoStacking);
}

void stackNewCone(bool waite=false, bool safeAtEnd=bIfiAutonomousMode) {	//TODO: liftTarget and liftRelease
	if (!(SONAR_STACKING && coneSonar[robot]>=dgtl1)) {
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
