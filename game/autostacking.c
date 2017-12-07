#include "audioControl.c"
#include "lift.c"

const float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_LIFT);

int numCones = 0; //current number of stacked cones
int liftTarget;
bool stacking = false;	//whether the robot is currently in the process of stacking

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_MIN], liftPos[L_MAX]);
}

void stackNewCone(bool wait=false) {	//TODO: liftTarget and liftRelease
	float stackHeight = CONE_HEIGHT * adjustedNumCones();

	liftTarget = calcLiftTargetForHeight(stackHeight);
	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}

task autoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		setLiftTargetAndPID(liftTarget);
		while (getPosition(lift) < liftTarget) EndTimeSlice();	//wait for lift to move to stacking position

		setFbState(STACK);
		waitForLiftingToFinish();

		if (numCones>=MAX_NUM_CONES-1 && HOLD_LAST_CONE) {
			lift.activelyMaintining = false;	//passively maintains lift position
			lift.stillSpeedReversed = false;
		}
		else {
			setLiftState(L_DEF);

			while (getPosition(lift) > liftTarget-100/L_CORR_FCTR) EndTimeSlice();
			setFbState(FB_DEF);
		}

		stacking = false;
		numCones++;
		speakNum(numCones);
	}
}
