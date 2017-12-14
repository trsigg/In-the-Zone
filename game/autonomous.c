#include "autostacking.c"
#include "mobileGoal.c"


//#region perparation
void prepareForAuton() {
	resetLiftEncoders();
	stopLiftTargeting();
	startTask(autoStacking);
	setPower(fourBar, FB_STILL_SPEED)
	numCones = 0;
}
//#endregion

//#region complex maneuvers
void alignToLine(int power=60, int brakePower=10, int brakeDuration=250) {	//brakepower is absolute value (sign automatically determined)
	long leftTimer, rightTimer;
	setDrivePower(drive, power, power);

	int leftProgress=0, rightProgress=0;	//0 - moving
	                                      //1 - braking
	                                      //2 - finished

	while (leftProgress<2 || rightProgress<2) {
		if (leftProgress == 0) {
			if (SensorValue[LEFT_LINE] < L_LINE_THRESHOLD) {
				leftTimer = resetTimer();
				setLeftPower(drive, -brakePower * sgn(power));
				leftProgress = 1;
			}
		} else if (leftProgress == 1) {
			if (time(leftTimer) >= brakeDuration) {
				setLeftPower(drive, 0);
				leftProgress = 2;
			}
		}

		if (rightProgress == 0) {
			if (SensorValue[RIGHT_LINE] < R_LINE_THRESHOLD) {
				rightTimer = resetTimer();
				setRightPower(drive, -brakePower * sgn(power));
				rightProgress = 1;
			}
		} else if (rightProgress == 1) {
			if (time(rightTimer) >= brakeDuration) {
				setRightPower(drive, 0);
				rightProgress = 2;
			}
		}

		EndTimeSlice();
	}
}

void turnToLine(bool parallelToLine, int power, int brakePower=20, int brakeDuration=250) {
	setDrivePower(drive, power, -power);

	if (parallelToLine)
		while (SensorValue[LEFT_LINE]<L_LINE_THRESHOLD && SensorValue[RIGHT_LINE]<R_LINE_THRESHOLD) EndTimeSlice();
	else
		while (SensorValue[BACK_LINE] < B_LINE_THRESHOLD) EndTimeSlice();

	setDrivePower(drive, -brakePower*sgn(power), brakePower*sgn(power));

	wait1Msec(brakeDuration);

	setDrivePower(drive, 0, 0);
}

void turnQuicklyToLine(bool clockwise, bool parallelToLine) {
	int direction = clockwise ? 1 : -1;
	turn(30 * direction);
	turnToLine(parallelToLine, 40*direction);
}

void driveForDuration(int duration, int beginPower=127, int endPower=0) {
	setDrivePower(drive, beginPower, beginPower);
	wait1Msec(duration);
	setDrivePower(drive, endPower, endPower);
}

void turnDriveTurn(int angle, int dist, int angle2=0) {
	turn(angle);
	driveStraight(dist);
	turn(angle2==0 ? -angle : angle2);
}

void driveAndGoal(int dist, bool in, bool stackCone=false, bool quadRamp=false, int intakeDelay=250) {
	moveGoalIntake(in, true);

	if (in)
		wait1Msec(intakeDelay);

	if (quadRamp)
		driveStraight(dist, true, 50, 120, -20, 250, 20, false);
	else
		driveStraight(dist, true);

	waitForMovementToFinish(goalIntake);

	if (stackCone) stackNewCone();

	while (driveData.isDriving || (stacking && stackCone)) EndTimeSlice();
}
//#endregion

//#region routine portions
void scoreGoal(bool retract=true, bool twentyPt=true) {	//lined up to 10pt bar -> behind 10pt bar (TODO: make retract 2nd arg)
	if (twentyPt)
		driveForDuration(1500, 127, 20);
	else
		driveForDuration(750, 60, 15);

	moveGoalIntake(false);	//extend goal intake

	driveForDuration(250);	//push goal to back of zone
	driveStraight(twentyPt ? -17 : -7);

	numCones = 0;
	if (retract) moveGoalIntake(true);	//retract goal intake
}

void sideGoal(bool retract=true, bool twentyPt=true, bool middle=true) {	//touching bar, aligned with goal -> behind 10pt bar
	if (lift.posPID.target<liftPos[L_SAFE] || lift.moving!=TARGET)
		moveLiftToSafePos();

	//pick up side goal
	driveAndGoal(15, false, false, true);
	driveStraight(30);

	//position robot so it is ready to outtake goal into 20pt zone
	driveAndGoal(-42, true, true);

	moveLiftToSafePos();

	turn(-45);	//turnDriveTurn?
	driveStraight(middle ? -23 : -10);
	turn(-90);

	scoreGoal(retract, twentyPt);
}

void middleGoal(bool left, bool twentyPt, bool center=true) {		//aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);

	moveGoalIntake(false);

	driveStraight(40);

	driveAndGoal(-30, true);
	if (twentyPt) stackNewCone();	//preload

	if (center) {
		turn(-90 * direction);
		driveStraight(-15);
		turn(-100 * direction);
	}
	else {
		turn(-180);
		driveForDuration(1250, 40);	//align to 10pt bar
	}

	while (stacking && twentyPt) EndTimeSlice();

	moveLiftToSafePos();
	scoreGoal(true, twentyPt);

	if (twentyPt) {
		/*driveForDuration(750, -127);	//back out of 20pt zone
		wait1Msec(750);*/
		driveForDuration(1250, 40);	//align against 10pt bar
	}
	//alignToLine(-60);	//align to tape (TODO: rename fn?)
	driveStraight(-7);
}
//#endregion

//#region routines
task skillz() {
	turnDefaults.reversed = false;

	middleGoal(true, true);	//near left middle goal

	turnDriveTurn(-95, GOAL_TO_MID_DIST);	//TODO: turnToLine() (& similar below)

	middleGoal(false, false, false);	//near right middle goal

	/*turn(-90);
	driveStraight(15);
	turn(-90);*/
	turnDriveTurn(-150, 5, -30);

	//far right middle goal
	driveStraight(60);	//push aside cones
	driveAndGoal(-GOAL_TO_MID_DIST, false);

	driveStraight(20);

	driveAndGoal(30, true);

	turnDriveTurn(-90, GOAL_TO_MID_DIST);

	scoreGoal();

	turnDriveTurn(-90, GOAL_TO_MID_DIST);

	middleGoal(false, false, false);

	turnDriveTurn(-90, 20, -45);

	turnDefaults.reversed = true;
	sideGoal(true, false, false);

	/*turnDefaults.reversed = true;	//TODO: unreverse

	sideGoal();

	driveForDuration(1000, -127);	//back over poles
	driveForDuration(750, 60);	//run into pole to align

	driveStraight(-1);
	turn(-45);
	driveStraight(27);
	turn(-90);

	driveAndGoal(30, false);*/
}

task altSkillz() {
	turnDefaults.reversed = false;

	middleGoal(false, true);

	turnDriveTurn(-90, 25 /*TODO: const (like goalToMid)*/);
}

task sideGoalTask() {
	sideGoal(true);
	//move lift out of way of intake
	/*setPower(coneIntake, 60);
	setChainBarState(STACK);
	setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);

	driveStraight(50, true);

	while (getPosition(lift) < liftPos[L_SAFE]) EndTimeSlice();
	setPower(coneIntake, INTAKE_STILL_SPEED);

	while (driveData.isDriving) EndTimeSlice();

	driveAndGoal(-20, false);

	turn(-45);
	driveStraight(10);

	moveGoalIntake(true, true);
	wait1Msec(250);
	turn(-45);

	driveStraight(-50);

	turn(-45);
	driveStraight(-23);
	turn(-90);
	driveForDuration(1500, 127, 20);

	moveGoalIntake(false);	//extend goal intake

	driveForDuration(250);	//push goal to back of zone
	driveForDuration(1000, -127);	//remove goal from intake

	numCones = 0;*/
}
//#endregion
