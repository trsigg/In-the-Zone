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

void waitForLiftingToFinish(bool waitForFB=true, bool waitForLift=true, int timeout=100, float fbMargin=100/FB_CORR_FCTR, float liftMargin=75/L_CORR_FCTR) {
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!(errorLessThan(lift, liftMargin) || errorLessThan(fourBar, fbMargin)))
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

void executeLiftManeuvers(bool autoStillSpeed=true) {
	handleEncoderCorrection();

	if (autoStillSpeed && errorLessThan(lift, L_AUTO_SS_MARGIN/L_CORR_FCTR) && lift.activelyMaintining && lift.posPID.target<=liftPos[L_FIELD])
		setPower(lift, LIFT_STILL_SPEED * (lift.posPID.target<=liftPos[L_FIELD] ? -1 : 1));
	else
		maintainTargetPos(lift, debugParameters[0]);

	if (autoStillSpeed && errorLessThan(fourBar, FB_AUTO_SS_MARGIN/FB_CORR_FCTR) && fourBar.activelyMaintining)
		setPower(fourBar, FB_STILL_SPEED);	//TODO: pos dependent?
	else
		maintainTargetPos(fourBar, debugParameters[2]);
}

void stopLiftTargeting() {
	stopTargeting(lift);
	stopTargeting(fourBar);
	setPower(lift, 0);
	setPower(fourBar, 0);
}

//#region lift
void setLiftPIDmode(bool up) {	//up is true for upward movement consts, false for downward movement ones.
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
void setFbPIDmode(bool up) {	//up is true for upward movement consts, false for downward movement ones.
	if (up)
		setTargetingPIDconsts(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 1.3*FB_CORR_FCTR);	//0.37, 0.002, 1.6
	else
		setTargetingPIDconsts(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 1.3*FB_CORR_FCTR);
}

void setFbTargetAndPID(int target, bool resetIntegral=true) {	//sets four bar target and adjusts PID consts
	if (MULTIPLE_PIDs) {
		if (getPosition(fourBar) < target)
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
//#endregion

void moveLiftToSafePos(bool wait=true) {	//TODO: fb?
	setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);
	setFbState(FB_SAFE);

	if (wait)	//TODO: ensure fb in correct position?
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}
