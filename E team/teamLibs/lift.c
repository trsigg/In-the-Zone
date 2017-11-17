#include "config.c"


bool fielding = true;	//whether robot is intaking cones from the driver load or field

void setLiftControlMode(bool field) {
	if (field) {
		configureButtonInput(lift, f_liftUpBtn, f_liftDownBtn);
		configureButtonInput(chainBar, f_chainOutBtn, f_chainInBtn);
	}
	else	{	//driver load mode
		configureButtonInput(lift, d_liftUpBtn, d_liftDownBtn);
		configureButtonInput(chainBar, d_chainOutBtn, d_chainInBtn);
	}

	configureBtnDependentStillSpeed(lift, LIFT_STILL_SPEED);
	configurePosDependentStillSpeed(chainBar, CHAIN_STILL_SPEED, chainPos[VERT]);
}

//#region sensors
void resetLiftEncoders() {
	resetEncoder(lift);
	resetEncoder(chainBar);
}

void handleEncoderCorrection() {
	if (L_USING_ENC)
		correctEncVal(lift);

	if (CH_USING_ENC)
		correctEncVal(chainBar);
}
//#endregion

//#region position targeting
void waitForLiftingToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=100, float chainMargin=20, float liftMargin=200) {	//fctr
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(chainBar, chainMargin) && waitForChain ||
				!errorLessThan(lift, liftMargin) && waitForLift)
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

void executeLiftManeuvers(bool autoStillSpeed=true) {
	handleEncoderCorrection();

	if (autoStillSpeed && errorLessThan(chainBar, CH_AUTO_SS_MARGIN/CH_CORR_FCTR) && chainBar.activelyMaintining)
		setPower(chainBar, CHAIN_STILL_SPEED * (chainBar.posPID.target<=chainPos[VERT] ? 1 : -1));
	else
		maintainTargetPos(chainBar, debugParameters[1]);

	if (autoStillSpeed && errorLessThan(lift, L_AUTO_SS_MARGIN) && lift.activelyMaintining && lift.posPID.target<=liftPos[L_FIELD])	//TODO: fctr
		setPower(lift, LIFT_STILL_SPEED * (lift.posPID.target<=liftPos[L_FIELD] ? -1 : 1));
	else
		maintainTargetPos(lift, debugParameters[0]);
}

void stopLiftTargeting() {
	stopTargeting(chainBar);
	setPower(chainBar, 0);

	stopTargeting(lift);
	setPower(lift, 0);
}

  //#subregion lift
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
  //#endsubregion

  //#subregion chain bar
void setChainBarPIDmode(bool low) {	//	low should be true for targets below VERT
	if (low)
		initializeTargetingPID(chainBar, 2.5, 0.01, 7, 10);
	else
		initializeTargetingPID(chainBar, 2.5, 0.01, 7, 10);
}

void setChainBarTargetAndPID(int target, bool resetIntegral=true) {
	if (MULTIPLE_PIDs)
		if (target > chainPos[VERT])
			setChainBarPIDmode(true);
		else
			setChainBarPIDmode(false);

	setTargetPosition(chainBar, target, resetIntegral);
}

void setChainBarState(chainState state) {
	if (state == CH_DEF)
		setChainBarState(fielding ? CH_FIELD : CH_SAFE);
	else
		setChainBarTargetAndPID(chainPos[state]);
}
  //#endsubregion
//#endregion

void moveLiftToSafePos(bool wait=true) {
	setChainBarState(STACK);
	setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);

	if (wait)
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}
