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
	setPower(coneIntake, -127);
	setLiftTargetAndPID(liftAngle1, false);
	waitForLiftingToFinish(true, false, OUTTAKE_DURATION);
}

task autoStacking() {
	bool useOffset;
	bool liftEarly;

	while (true) {
		while (!stacking) EndTimeSlice();

		useOffset = (numCones >= NO_OFFSET_CONES);
		liftEarly = !fielding && (numCones < D_LIFT_EARLY_CONES);

		//intake cone
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(numCones<RECKLESS_CONES ? STACK : CH_SAFE);
		setLiftTargetAndPID(useOffset ? liftAngle1 : liftAngle2);

		while (!errorLessThan(lift, 200/L_CORR_FCTR)) EndTimeSlice();
		if (numCones >= RECKLESS_CONES) setChainBarState(STACK);
		waitForLiftingToFinish(false);
		if (useOffset) setLiftTargetAndPID(liftAngle2/*, false*/);

		waitForLiftingToFinish(true, true, 250);

		if (numCones < MAX_NUM_CONES-1) {
			expelCone();
			stacking = false;
			numCones++;

			if (liftEarly) setLiftState(L_DEF);
			setChainBarState(CH_DEF);
			setPower(coneIntake, 0);

			//return to ready positions
			while (getPosition(chainBar) < chainPos[CH_SAFE]-(fielding ? 0 : 100/CH_CORR_FCTR)) EndTimeSlice();
			if (!liftEarly) setLiftState(L_DEF);
		}
		else {
			if (!HOLD_LAST_CONE)
				expelCone();
			stacking = false;
			numCones++;

			lift.activelyMaintining = false;	//allows lift to fall down on stack
			lift.stillSpeedReversed = true;
		}

		speakNum(numCones);
	}
}
