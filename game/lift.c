#include "..\config\config.c"
#include "..\lib\pneumaticGroup.c"


bool fielding = true;	//whether robot is intaking cones from the driver load or field
bool fbUpAfterLiftManeuver;	//whether four bar should raise after current lift maneuver (TODO: down option as well?)
int numCones = 0; //current number of stacked cones (mostly used in autostacking) - TODO: move

//#region sensors
void resetLiftEncoders() {
	resetEncoder(lift);
	#ifndef PNEUMATIC
		resetEncoder(fourBar);
	#endif
}

void handleEncoderCorrection() {
	if (liftSensor[robot] >= dgtl1)
		correctEncVal(lift);

	#ifndef PNEUMATIC
		if (fbSensor[robot] >= dgtl1)
			correctEncVal(fourBar);
	#endif
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

void setLiftState(liftState state, bool useManeuver=false) {
	if (state == L_DEF)
		setLiftState(fielding ? L_FIELD : D_LOAD, useManeuver);
	else if (useManeuver)
		createManeuver(lift, liftPos[state]);
	else
		setLiftTargetAndPID(liftPos[state]);
}

bool liftUntilSonar(bool up, int timeout=10, int power=127, int lowPower=60, int quitMargin=75) {	//if obstructed is true, will wait until lift is obstructed
	stopAutomovement(lift);
	power = abs(power) * (up ? 1 : -1);
	lowPower = abs(lowPower) * (up ? 1 : -1);

	bool abort = false;
	int dangerPos = liftPos[ up ? L_MAX : L_MIN ];

	long timer = resetTimer();

	while (time(timer) < timeout && !abort) {
		if (sonarFartherThan(coneSonar[robot], coneSonarThresh[robot], false) == up) {
			setPower(lift, lowPower);
		}
		else {
			setPower(lift, power);
			timer = resetTimer();
		}

		if (abs(getPosition(lift) - dangerPos) < quitMargin)
			abort = true;

		EndTimeSlice();
	}
	if (up) { generalDebug[0] = SensorValue[ coneSonar[robot] ]; }
	setToStillSpeed(lift, false);

	return abort;
}

	//#subregion kinematics
const float heightOffset = sin((liftPos[L_ZERO] - liftPos[M_BASE_POS]) / RAD_TO_LIFT);

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / liftLen[robot] - heightOffset) + liftPos[L_ZERO],
						   liftPos[L_MIN], liftPos[L_MAX]);
}

float calcLiftHeight(int liftVal) {	//finds lift sensor val separated from liftVal by offset inches	- TODO: what?
	return 2 * liftLen[robot] * (sin((liftVal - liftPos[L_ZERO]) / RAD_TO_LIFT) + heightOffset);
}
	//#endsubregion
//#endregion

//#region four bar
bool fbUp = true;

void moveFourBar(bool up, bool runConcurrently=true, int power=127) {
	fbUp = up;

	if ((getPosition(fourBar) <= fbPos[FB_UP]) != up) {
		#ifdef PNEUMATIC
			setState(fourBar, !up, runConcurrently);
		#else
			fourBar.stillSpeedReversed = !up;
			power *= (up ? -1 : 1);

			if (fbSensor[robot] >= 0)
				createManeuver(fourBar, fbPos[up ? FB_UP : FB_DOWN], runConcurrently, power, );
			else
				moveForDuration(fourBar, power, fbMoveDuration[robot], runConcurrently);
		#endif
	}
}
//#endregion

//#region general maneuvers
void executeManeuvers(bool autoStillSpeed=true) {	//TODO: argument doesn't do anything right now
	handleEncoderCorrection();

	executeAutomovement(lift, debugParameters[0]);
	#ifndef PNEUMATIC
		executeAutomovement(fourBar, debugParameters[2]);
	#endif

	//I know, this isn't really part of the lift... (TODO: reposition)
	executeAutomovement(goalIntake);
	#ifdef ROLLER
		executeAutomovement(roller);
	#endif

	if (fbUpAfterLiftManeuver && (lift.moving==NO || lift.moving==TARGET && errorLessThan(lift, lift.waitErrorMargin))) {	//TODO: better targeting completion criterion?
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

	#ifndef PNEUMATIC
		stopAutomovement(fourBar);
	#endif
}
//#endregion

void moveLiftToSafePos(bool waite=true, bool moveFb=true) {
	if (lift.moving!=TARGET || lift.posPID.target<liftPos[L_SAFE])
			setLiftTargetAndPID(liftPos[L_SAFE] + 50/L_CORR_FCTR);

	if (moveFb) moveFourBar(true);

	if (waite)	//TODO: ensure fb in correct position?
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}

void liftToConeSafePos() {
	setLiftTargetAndPID(max(calcLiftTargetForHeight(coneHeight[robot] * numCones + 9), liftPos[L_SAFE]));
}
