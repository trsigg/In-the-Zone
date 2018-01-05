#include "autostacking.c"
#include "mobileGoal.c"
#include "testing.c"


//#region perparation
void prepareForAuton() {
	resetLiftEncoders();
	stopLiftTargeting();
	startTask(autoStacking);
	setPower(fourBar, FB_STILL_SPEED);
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

void turnToLine(bool parallelToLine, int power, int brakePower=20, int brakeDuration=250) {	//parallelToLine should reflect the robot's initial position
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

void alignToBar(bool forward=true, int duration=750) {
	int direction = (forward ? 1 : -1);
	driveForDuration(duration, 30*direction, 15*direction);
}

void turnDriveTurn(int angle, int dist, int angle2=0) {
	turn(angle);
	driveStraight(dist);
	turn(angle2==0 ? -angle : angle2);
}

void driveAndGoal(int dist, goalState state, bool stackCone=false, bool quadRamp=false, int intakeDelay=500) {
	moveLiftToSafePos();
	moveGoalIntake(state, true);

	if (state != OUT)
		wait1Msec(intakeDelay);

	if (quadRamp)
		driveStraight(dist, true, 30, 120, -20, 250, 20, false);
	else
		driveStraight(dist, true);

	waitForMovementToFinish(goalIntake);

	if (stackCone) stackNewCone();

	while (driveData.isDriving /*|| (stacking && stackCone)*/) EndTimeSlice();
}
//#endregion

//#region routine portions
void scoreGoal(bool twentyPt=true, bool align=true, bool intakeFully=true) {	//behind 10pt bar -> aligned with tape (approx)
	if (twentyPt)
		driveForDuration(1000, 90, 20);	//drive over bar
	else
		alignToBar();

	moveGoalIntake(OUT);

	if (twentyPt) {
		driveForDuration(250);	//push goal to back of zone
		driveForDuration(250, -127);
	}
	else {
		driveStraight(-10);	//back out
	}

	moveGoalIntake(intakeFully ? IN : MID, true);
	while (getPosition(goalIntake) < goalPos[MID]) EndTimeSlice();

	if (twentyPt)
		driveStraight(-17);	//back out

	if (align) {
		alignToBar();
		driveStraight(-BAR_TO_LINE_DIST);
	}

	numCones = 0;
}

void sideGoal(bool twentyPt=true, bool middle=false, bool reversed=false, bool align=true, bool intakeFully=true) {	//touching bar, aligned with goal -> aligned with tape (approx)
	int direction = (reversed ? -1 : 1);
	moveLiftToSafePos();

	//pick up side goal
	driveAndGoal(15, OUT, false, true);
	driveStraight(30);

	//position robot facing middle of 10pt bar
	driveAndGoal(-42, IN, true);

	turn(-direction * 45);	//turnDriveTurn?
	driveStraight(middle||twentyPt ? -25 : -12);
	turn(-direction * 90);

	while (stacking) EndTimeSlice();
	moveLiftToSafePos();

	scoreGoal(twentyPt, align, intakeFully);
}

void middleGoal(bool left, bool twentyPt=true, bool middle=false, bool align=true, bool intakeFully=true) { //aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);                                                                          //if twentyPt and middle are true, assumes starting from bar (not tape)

	moveLiftToSafePos();
	moveGoalIntake(OUT);	//TODO: check if intake is out?

	driveStraight(35 - (twentyPt&&middle ? 0 : BAR_TO_LINE_DIST));

	driveAndGoal(-23.5, IN);
	if (twentyPt) {	//preload
		stackNewCone();
		moveLiftToSafePos(false);
	}

	if (middle || twentyPt) {
		turn(-90 * direction);
		driveStraight(-GOAL_TO_MID_DIST-3);
		turn(-90 * direction);
	}
	else {
		turn(-175);
		//driveForDuration(1250, 40);	//align to 10pt bar
	}

	while (stacking) EndTimeSlice();

	moveLiftToSafePos();
	scoreGoal(twentyPt, align, intakeFully);
}
//#endregion

//#region routines
task skillz() {
	turnDefaults.reversed = false;

	middleGoal(true, true, true);	//near left middle goal to 20Pt zone

	moveGoalIntake(OUT, true);
	turnDriveTurn(-90, GOAL_TO_MID_DIST-1, -90);	//TODO: turnToLine() (& similar below)
	//alignToBar(false, 1000);

	middleGoal(false, false, false, false);	//near right middle goal to right 10pt

	turn(-175);
	moveGoalIntake(OUT);
	alignToBar(false, 1500);

	//far right middle goal to 20pt
	driveStraight(75);	//previously 30 then 45
	/*driveStraight(60);	//push aside cones
	moveGoalIntake(-GOAL_TO_MID_DIST, OUT);

	driveStraight(25);*/

	driveAndGoal(20/*43*/, IN);

	turnDriveTurn(-90, GOAL_TO_MID_DIST-5);

	scoreGoal();

	//far left middle goal to left 10pt
	moveGoalIntake(OUT, true);
	turnDriveTurn(-90, GOAL_TO_MID_DIST-3, -90);
	//alignToBar(false, 1500);

	middleGoal(false, false, true);

	turnDriveTurn(-90, 25, -45);

	sideGoal(false, false, true);	//far left side goal to left 10pt
}

task altSkillz() {
	turnDefaults.reversed = false;

	middleGoal(false);

	turnDriveTurn(-90, 25 /*TODO: const (like goalToMid)*/);


}

task antiMark() {
	driveStraight(15);
	turn(45);
	driveStraight(55);
	driveAndGoal(-30, OUT);
	turn(45);
	driveStraight(15);
	moveGoalIntake(IN, false);
	stackNewCone();
}
//#endregion

task autonUpdateTask() {
	while (true) {
		executeManeuvers();
		logSensorVals();
		EndTimeSlice();
	}
}
