#include "..\config\config.c"


bool fielding = true;	//whether robot is intaking cones from the driver load or field
bool fbUpAfterLiftManeuver;	//whether four bar should raise after current lift maneuver (TODO: down option as well?)

//#region sensors
void resetLiftEncoders() {
	resetEncoder(lift);
	resetEncoder(fourBar);
}

void handleEncoderCorrection() {
	if (LIFT_SENSOR >= dgtl1)
		correctEncVal(lift);

	if (FB_SENSOR >= dgtl1)
		correctEncVal(fourBar);
}
//#endregion

//#region lift
void setLiftPIDmode(bool up) {	//up is true for upward movement consts, false for downward movement ones
	if (MULTIPLE_PIDs)
		if (up)
			setTargetingPIDconsts(lift, 0.35*L_CORR_FCTR, 0.005*L_CORR_FCTR, 0.7*L_CORR_FCTR);	//0.37, 0.002, 1.6
		else
			setTargetingPIDconsts(lift, 0.35*L_CORR_FCTR, 0.005*L_CORR_FCTR, 0.7*L_CORR_FCTR);
}

void setLiftTargetAndPID(int target, bool resetIntegral=true) {	//sets lift target and adjusts PID consts
	if (MULTIPLE_PIDs) {
		if (getPosition(lift) < target)
			setLiftPIDmode(true);
		else
			setLiftPIDmode(false);
	}

	setTargetPosition(lift, limit(target, liftPos[L_MIN], liftPos[L_MAX]), resetIntegral);
}

void setLiftState(liftState state) {
	if (state == L_DEF)
		setLiftState(fielding ? L_FIELD : D_LOAD);
	else
		setLiftTargetAndPID(liftPos[state]);
}

bool liftUntilSonar(bool obstructed, bool up, int additionalTime=75, int power=127, int quitMargin=75) {	//if obstructed is true, will wait until lift is obstructed
	setPower(lift, abs(power)*(up ? 1 : -1));

	bool abort = false;
	int dangerPos = liftPos[ up ? L_MAX : L_MIN ];

	while (xor(sonarFartherThan(CONE_SONAR, CONE_SONAR_THRESH), !obstructed) && !abort) {
		if (abs(getPosition(lift) - dangerPos) < quitMargin)
			abort = true;
		EndTimeSlice();
	}

	if (!abort) wait1Msec(additionalTime);

	setGroupToStillSpeed(lift);

	return abort;
}

	//#subregion kinematics
const float heightOffset = sin((liftPos[L_ZERO] - liftPos[M_BASE_POS]) / RAD_TO_LIFT);

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / LIFT_LEN - heightOffset) + liftPos[L_ZERO],
						   liftPos[L_MIN], liftPos[L_MAX]);
}

float calcLiftHeight(int liftVal) {	//finds lift sensor val separated from liftVal by offset inches	- TODO: what?
	return 2 * LIFT_LEN * (sin((liftVal - liftPos[L_ZERO]) / RAD_TO_LIFT) + heightOffset);
}
	//#endsubregion
//#endregion

//#region four bar
bool fbUp = false;

void setFbPIDmode(bool high) {	//high is true for targets above FB_SAFE
	if (MULTIPLE_PIDs)
		if (high)
			setTargetingPIDconsts(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 1.3*FB_CORR_FCTR);	//0.37, 0.002, 1.6
		else
			setTargetingPIDconsts(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 1.3*FB_CORR_FCTR);
}

void setFbTargetAndPID(int target, bool resetIntegral=true) {	//sets four bar target and adjusts PID consts
	if (MULTIPLE_PIDs) {
		if (target > fbPos[FB_SAFE])
			setFbPIDmode(true);
		else
			setFbPIDmode(false);
	}

	setTargetPosition(fourBar, target, resetIntegral);
}

void setFbState(fbState state) {
	if (state == FB_DEF)
		setFbState(fielding ? FB_FIELD : FB_SAFE);
	else
		setFbTargetAndPID(fbPos[state]);
}

void moveFourBar(bool up, bool runConcurrently=true, int power=127) {
	fbUp = up;
	fourBar.stillSpeedReversed = !up;
	moveForDuration(fourBar, power*(up ? 1 : -1), FB_MOVE_DURATION, runConcurrently);
}
//#endregion

//#region general maneuvers
void executeManeuvers(bool autoStillSpeed=true) {	//TODO: argument doesn't do anything right now
	handleEncoderCorrection();

	executeAutomovement(lift, debugParameters[0]);
	executeAutomovement(fourBar, debugParameters[2]);
	executeAutomovement(goalIntake);	//I know, this isn't really part of the lift... (TODO: reposition)

	if (fbUpAfterLiftManeuver && (lift.moving==NO || lift.moving==TARGET && errorLessThan(lift, lift.waitErrorMargin))) {	//TODO: better targeting completion criterion?
		if (FB_SENSOR >= 0)
			setFbState(FB_SAFE);
		else
			moveFourBar(true);

		fbUpAfterLiftManeuver = false;
	}
}

void waitForLiftingToFinish(int timeout=100) {	//TODO: delete as soon as possible
	waitForMovementToFinish(lift, timeout);
	waitForMovementToFinish(fourBar);
}

void stopLiftTargeting() {
	stopAutomovement(lift);
	stopAutomovement(fourBar);
}
//#endregion

void moveLiftToSafePos(bool wait=true) {
	if (getPosition(lift)<liftPos[L_SAFE]) {
		if (lift.moving!=TARGET || lift.posPID.target<liftPos[L_SAFE])
			setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);
	}
	else {	//passively hold lift up
		stopAutomovement(lift);
		setPower(lift, LIFT_STILL_SPEED);
		lift.stillSpeedReversed = false;
	}

	if (FB_SENSOR >= 0)	//and fb not in?
		setFbState(FB_SAFE);
	else if (!fbUp)
		moveFourBar(true);

	if (wait)	//TODO: ensure fb in correct position?
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}
