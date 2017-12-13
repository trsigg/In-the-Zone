#include "audioControl.c"
#include "lift.c"

const float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_LIFT);

int numCones = 0; //current number of stacked cones
int liftTarget, liftRelease;
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

	liftTarget = calcLiftTargetForHeight(stackHeight + L_OFFSET);	//TODO: noOffsetCones, etc?
	liftRelease = calcLiftTargetForHeight(stackHeight);
	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}

task autoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		setLiftTargetAndPID(liftTarget);
		if (FB_SENSOR >= 0) setFbState(FB_SAFE);
		while (getPosition(lift) < liftTarget) EndTimeSlice();	//wait for lift to move to stacking position

		if (FB_SENSOR >= 0)
			setFbState(STACK);
		else
			moveFourBar(true);

		waitForMovementToFinish();

		if (numCones>=MAX_NUM_CONES-1 && HOLD_LAST_CONE) {
			stopAutomovement(lift);	//passively maintains lift position
			lift.stillSpeedReversed = false;
		}
		else {
			setLiftState(L_DEF);

			while (getPosition(lift) > liftRelease) EndTimeSlice();
			if (FB_SENSOR >= 0)
				setFbState(FB_DEF);
			else
				moveFourBar(false);
		}

		stacking = false;
		numCones++;
		speakNum(numCones);
	}
}
