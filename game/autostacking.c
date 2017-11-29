#include "audioControl.c"
#include "lift.c"

const float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_LIFT);

int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_MIN], liftPos[L_MAX]);
}

void stackNewCone(bool wait=false) {
	float stackHeight = CONE_HEIGHT * adjustedNumCones();

	setLiftTargetAndPID(calcLiftTargetForHeight(stackHeight));
	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}

void expelCone() {	//should be called after stacking cone
	setPower(sideRollers, 127);
	wait1Msec(OUTTAKE_DURATION);
	setPower(sideRollers, 0);
}

task autoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		waitForLiftingToFinish(400);	//wait for lift to move to stacking position

		expelCone();
		stacking = false;
		numCones++;

		if (numCones==MAX_NUM_CONES && HOLD_LAST_CONE) {
			lift.activelyMaintining = false;	//passively maintains lift position
			lift.stillSpeedReversed = false;
		}
		else {
			setLiftState(L_DEF);
		}

		speakNum(numCones);
	}
}
