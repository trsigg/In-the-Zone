#include "..\config\config.c"


bool fielding = true;	//whether robot is intaking cones from the driver load or field

//#region sensors
void resetLiftEncoders() {
	resetEncoder(lift);
}

void handleEncoderCorrection() {
	if (L_USING_ENC)
		correctEncVal(lift);
}
//#endregion

//#region position targeting
void waitForLiftingToFinish(int timeout=100, float liftMargin=200/L_CORR_FCTR) {
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(lift, liftMargin))
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
}

void stopLiftTargeting() {
	stopTargeting(lift);
	setPower(lift, 0);
}

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

void moveLiftToSafePos(bool wait=true) {
	setState(fourBar, false);
	setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);

	if (wait)
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}
