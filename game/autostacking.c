#include "audioControl.c"
#include "lift.c"

const float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_LIFT);

int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
float liftAngle1, liftAngle2;	//the target angles of lift sections during a stack maneuver

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_MIN], liftPos[L_MAX]);
}

void stackNewCone(bool wait=false) {
	float stackHeight = CONE_HEIGHT * adjustedNumCones();

	liftAngle1 = calcLiftTargetForHeight(stackHeight + LIFT_OFFSET);
	liftAngle2 = calcLiftTargetForHeight(stackHeight);
	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}

void expelCone() {	//should be called after stacking cone
	setState(coneIntake, false);
	setLiftTargetAndPID(liftAngle1, false);
	waitForLiftingToFinish(OUTTAKE_DURATION);
}

task autoStacking() {
	bool useOffset;
	bool liftEarly;

	while (true) {
		while (!stacking) EndTimeSlice();

		useOffset = (numCones >= NO_OFFSET_CONES);
		liftEarly = !fielding && (numCones < D_LIFT_EARLY_CONES);

		//move to desired location
		setLiftTargetAndPID(useOffset ? liftAngle1 : liftAngle2);

		while (!errorLessThan(lift, 200/L_CORR_FCTR)) EndTimeSlice();
		setState(fourBar, true);
		if (useOffset) setLiftTargetAndPID(liftAngle2/*, false*/);

		waitForLiftingToFinish(FB_LIFT_DURATION);

		expelCone();
		stacking = false;
		numCones++;

		if (numCones<MAX_NUM_CONES || !HOLD_LAST_CONE) {
			if (liftEarly) {
				setLiftState(L_DEF);
				waitForLiftingToFinish();
				setState(fourBar, false);
			}
			else {
				setState(fourBar, false);
				wait1Msec(FB_LIFT_DURATION);
				setLiftState(L_DEF);
			}
		}
		else {
			lift.activelyMaintining = false;	//passively maintains lift position
			lift.stillSpeedReversed = false;
		}

		setState(coneIntake, true);	//TODO: put this before lifting?
		speakNum(numCones);
	}
}
