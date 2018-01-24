#include "autostacking.c"
#include "mobileGoal.c"
#include "testing.c"


int autonDebug;
bool variant = false;
int numRetries = 0;

//#region perparation
void prepareForAuton() {
	resetLiftEncoders();
	stopLiftTargeting();
	startAutoStacking();
	setPower(fourBar, FB_STILL_SPEED);
	numCones = 0;

	if (!USE_ENC_CORR) {
		driveDefaults.kP_c = 0;
		driveDefaults.kI_c = 0;
		driveDefaults.kD_c = 0;
	}

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

void driveToSonarDist(int dist, int power=30, int brakePower=10, int brakeDuration=150) {
	setDrivePower(drive, power, power);
	while (SensorValue[FRONT_SONAR]>dist || SensorValue[FRONT_SONAR]==-1) EndTimeSlice();
	setDrivePower(drive, -brakePower, -brakePower);
	wait1Msec(brakeDuration);
	setDrivePower(drive, 0, 0);
}

void driveAndSonar(int driveDist, int sonarDist, int power=30, int brakePower=10, int brakeDuration=150) {
	driveStraight(driveDist);
	driveToSonarDist(sonarDist, power, brakePower, brakeDuration);
}

void driveForDuration(int duration, int beginPower=127, int endPower=0) {
	setDrivePower(drive, beginPower, beginPower);
	wait1Msec(duration);
	setDrivePower(drive, endPower, endPower);
}

void alignToBar(bool forward=true, int duration=600) {
	int direction = (forward ? 1 : -1);
	driveForDuration(duration, 35*direction, 15*direction);
}

void turnDriveTurn(int angle, int dist, int angle2=0) {
	turn(angle);
	driveStraight(dist);
	turn(angle2==0 ? -angle : angle2);
}

void driveAndGoal(int dist, goalState state, bool stackCone=false, bool quadRamp=false, int intakeDelay=750) {
	moveLiftToSafePos();
	moveGoalIntake(state, true);

	if (state!=OUT && dist<0)
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
	wait1Msec(200);	//while (getPosition(goalIntake) < goalPos[MID]) EndTimeSlice();

	if (twentyPt)
		driveStraight(-17);	//back out

	if (align) {
		alignToBar();
		driveStraight(-BAR_TO_LINE_DIST);
	}

	if (!variant) waitForMovementToFinish(goalIntake);	//temp
	if (isMobileGoalLoaded() && numRetries<MAX_GOAL_RETRIES && RETRY_GOAL_FAILS) {
		numRetries++;
		scoreGoal(twentyPt, align, intakeFully);
	}
	else {
		numCones = 0;
		numRetries = 0;
	}
}

void sideGoal(bool twentyPt=true, bool middle=false, bool reversed=false, bool align=true, bool startingFromBar=true, bool intakeFully=true) {	//touching bar, aligned with goal -> aligned with tape (approx)
	int direction = (reversed ? -1 : 1);
	moveLiftToSafePos();

	//pick up side goal
	driveAndGoal((startingFromBar ? 15 : 10), OUT, false, true);
	driveStraight(startingFromBar ? 30 : 20);

	//position robot facing middle of 10pt bar
	driveAndGoal(-42, IN);

	if (!(isMobileGoalLoaded() || SKILLZ_MODE) && ABORT_IF_NO_GOAL) {	//stop execution in case mobile goal not intaked
		playSound(soundLowBuzz);
		while (true) EndTimeSlice();
	}

	if (!SKILLZ_MODE) {
		turn(-direction * 45);	//turnDriveTurn?
		driveStraight(middle||twentyPt ? -30 : -12, true);
		while (driveData.totalDist < 5) EndTimeSlice();
		stackNewCone();
		while (driveData.isDriving) EndTimeSlice();
		turn(-direction * 90);

		while (stacking) EndTimeSlice();
		moveLiftToSafePos();

		scoreGoal(twentyPt, align, intakeFully);
	}
	else {
		moveGoalIntake(OUT, true);
		turn(-180);
		waitForMovementToFinish(goalIntake);
		driveStraight(-10);
		turn(160);
		driveForDuration(2000);
	}
}

void middleGoal(bool left, bool twentyPt=true, bool middle=false, bool align=true, bool intakeFully=true) { //aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);                                                                          //if twentyPt and middle are true, assumes starting from bar (not tape)

	moveLiftToSafePos();
	moveGoalIntake(OUT);	//TODO: check if intake is out?

	driveStraight(35 - (twentyPt&&middle ? 0 : BAR_TO_LINE_DIST));

	driveAndGoal(-29, IN);
	if (twentyPt) {	//preload
		stackNewCone();
		moveLiftToSafePos(false);
	}

	if (middle || twentyPt) {
		turn(-90 * direction);
		driveStraight(-GOAL_TO_MID_DIST);
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

	turnDriveTurn(-90, GOAL_TO_MID_DIST-1.5, -90);	//TODO: turnToLine() (& similar below)
	moveGoalIntake(OUT);
	//alignToBar(false, 1000);

	middleGoal(false, false, false/*, false*/);	//near right middle goal to right 10pt

	turn(-180);
	moveGoalIntake(OUT, true);
	alignToBar(false, 1500);

	//far right middle goal to 20pt
	driveStraight(75);	//previously 30 then 45
	/*driveStraight(60);	//push aside cones
	moveGoalIntake(-GOAL_TO_MID_DIST, OUT);

	driveStraight(25);*/

	driveAndGoal(40/*43*/, IN);

	turn(-90);
	driveStraight(GOAL_TO_MID_DIST-3);
	//moveGoalIntake(MID);	//to push cones to side
	turn(90);
	moveGoalIntake(IN);

	scoreGoal();

	//far left middle goal to left 10pt
	moveGoalIntake(OUT, true);
	turnDriveTurn(-90, GOAL_TO_MID_DIST-1.5, -85);
	//alignToBar(false, 1500);

	middleGoal(false, false/*, !PARK_IN_SKILLS*/);	//temp

	if (PARK_IN_SKILLS) {
		turn(-140);
		driveForDuration(2000);
	}
	else {
		turnDriveTurn(-90, 18, -45);

		sideGoal(false, false, true, true, false);	//far left side goal to left 10pt
	}
}

task altSkillz() {
	turnDefaults.reversed = false;

	middleGoal(false);

	turnDriveTurn(-90, 25 /*TODO: const (like goalToMid)*/);


}

task antiMark() {
	//if (ANTI_MARK == 1) {
		wait1Msec(2000);
		driveStraight(10);
		turn(47);
		driveStraight(75);
		driveAndGoal(-45, OUT);
		turn(30);
		driveStraight(33);
		moveGoalIntake(IN, false);
		stackNewCone();
	/*}
	else {
		moveFourBar(true);
		setLiftTargetAndPID(2500);
		waitForLiftingToFinish();

		driveStraight(15);
		stopAutomovement(lift);
		setPower(lift, -15);
		wait1Msec(750);
		driveForDuration(500);
		setLiftTargetAndPID(2500);
	}*/
}
//#endregion

task autonUpdateTask() {
	while (true) {
		executeManeuvers();
		logSensorVals();
		EndTimeSlice();
	}
}

#ifdef RUN_AUTON_AS_MAIN
task main() {
#else
task autonomous() {
#endif
	prepareForAuton();
	handleTesting();
	startTask(autonUpdateTask);

	int sidePos = SensorValue[SIDE_POT];
	int modePos = SensorValue[MODE_POT];
	int autonTimer = resetTimer();

	turnDefaults.reversed = sidePos < SIDE_SWITCH_POS;	//TODO: put this val in config
	variant = abs(sidePos - SIDE_SWITCH_POS) < 1400;

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else if (modePos < 2030) {	//side goal
		bool twentyPt = modePos<450;
		sideGoal(twentyPt, false, false, false);

		if (variant) {	//drive to other side
			turnDriveTurn(90, (twentyPt ? 30 : 12), 90);
			driveStraight(60);
		}
	}
	else if (modePos < 3340) {	//defensive
		if (variant)
			startTask(antiMark);
		else
			driveForDuration(2000, 127);
	}

	autonDebug = time(autonTimer);
	while (true) EndTimeSlice();
}
