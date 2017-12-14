#include "..\config\config.c"


bool fielding = true;	//whether robot is intaking cones from the driver load or field

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

void executeManeuvers(bool autoStillSpeed=true) {	//TODO: argument doesn't do anything right now
	handleEncoderCorrection();

	executeAutomovement(lift, debugParameters[0]);
	executeAutomovement(fourBar, debugParameters[2]);
	executeAutomovement(goalIntake);	//I know, this isn't really part of the lift... (TODO: reposition)
}

void waitForLiftingToFinish(int timeout=100) {	//TODO: delete as soon as possible
	waitForMovementToFinish(lift, timeout);
	waitForMovementToFinish(fourBar);
}

void stopLiftTargeting() {
	stopAutomovement(lift);
	stopAutomovement(fourBar);
}

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

	setTargetPosition(lift, target, resetIntegral);
}

void setLiftState(liftState state) {
	if (state == L_DEF)
		setLiftState(fielding ? L_FIELD : D_LOAD);
	else
		setLiftTargetAndPID(liftPos[state]);
}
//#endregion

//#region four bar
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

void moveFourBar(bool up, bool runConcurrently=true) {
	moveForDuration(fourBar, 127*(up ? 1 : -1), FB_MOVE_DURATION, runConcurrently);
}
//#endregion

void moveLiftToSafePos(bool wait=true) {
	if (getPosition(lift) < liftPos[L_SAFE]) {
		setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);
	}
	else {	//passively hold lift up
		stopAutomovement(lift);
		lift.stillSpeedReversed = false;
	}

	if (FB_SENSOR >= 0)
		setFbState(FB_SAFE);
	else
		moveFourBar(true);

	if (wait)	//TODO: ensure fb in correct position?
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}
