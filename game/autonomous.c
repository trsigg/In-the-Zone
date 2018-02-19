#include "autostacking.c"
#include "mobileGoal.c"
#include "rollers.c"
#include "testing.c"


enum zoneType { FIVE, TEN, TWENTY };

int autonDebug[2];	// { time of auton routine, follower value on abort }
int autonTimer;
bool variant = false;
int numRetries = 0;

task autonUpdateTask() {
	while (true) {
		executeManeuvers();
		updateMotorConfig();
		logSensorVals();
		EndTimeSlice();
	}
}

//#region preparation
void prepareForAuton() {
	resetLiftEncoders();
	stopLiftTargeting();
	startAutoStacking();
	setToStillSpeed(roller);
	setPower(fourBar, FB_STILL_SPEED);
	numCones = 0;

	if (!USE_ENC_CORR) {
		driveDefaults.kP_c = 0;
		driveDefaults.kI_c = 0;
		driveDefaults.kD_c = 0;
	}

}
//#endregion

//#region conditional abort
void stopAllMotors() {	//TODO: reposition?
	for (int id=port1; id<=port10; id++) {
		motor[id] = 0;
	}
}

void maybeAbort() {	//stop execution in case mobile goal not intaked
	if (!(isMobileGoalLoaded() || SKILLZ_MODE) && ABORT_IF_NO_GOAL) {
		playSound(soundLowBuzz);
		stopTask(autonUpdateTask);
		stopAllMotors();

		while (true) EndTimeSlice();
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
	while (SensorValue[FRONT_SONAR]>dist || SensorValue[FRONT_SONAR]==-1) EndTimeSlice();	//TODO: use drive sonar & sonarFartherThan()
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

void driveAndGoal(int dist, goalState state, bool stackCone=false, bool quadRamp=false, int intakeDelay=500) {
	moveLiftToSafePos();
	moveGoalIntake(state, true);

	if (state!=OUT && dist<0)
		wait1Msec(intakeDelay);

	if (quadRamp)
		driveStraight(dist, true, 30, 120, -20, 250, 20, false);
	else
		driveStraight(dist, true);

	waitForMovementToFinish(goalIntake);

	if (stackCone) stackNewCone(false, true);

	while (driveData.isDriving /*|| (stacking && stackCone)*/) EndTimeSlice();
}
//#endregion

//#region routine portions
void scoreGoal(bool twentyPt=true, bool align=true, bool intakeFully=true) {	//behind 10pt bar -> aligned with tape (approx)
	if (twentyPt) {
		driveForDuration(1000, 90, 20);	//drive over bar
		moveGoalIntake(OUT);
		driveForDuration(250);	//push goal to back of zone
		driveForDuration(500, -127);
	}
	else {
		moveGoalIntake(OUT, true);
		alignToBar();
		waitForMovementToFinish(goalIntake);
		driveStraight(-10);	//back out
	}

	moveGoalIntake(intakeFully ? IN : MID/*, true*/);
	wait1Msec(200);	//while (getPosition(goalIntake) < goalPos[MID]) EndTimeSlice();

	if (twentyPt)
		driveStraight(-17);	//back out

	if (align) {
		alignToBar();
		driveStraight(-BAR_TO_LINE_DIST);
	}

	//waitForMovementToFinish(goalIntake);	//if something breaks with the goal intake, uncomment this line
	if (isMobileGoalLoaded() && numRetries<MAX_GOAL_RETRIES && RETRY_GOAL_FAILS) {
		numRetries++;
		playSound(soundBeepBeep);
		autonDebug[1] = SensorValue[GOAL_FOLLOWER];
		scoreGoal(twentyPt, align, intakeFully);
	}
	else {
		numCones = 0;
		numRetries = 0;
	}
}

void sideGoal(zoneType zone=TWENTY, bool middle=false, bool reversed=false, bool align=true, bool startingFromBar=true, bool fieldCones=false, bool intakeFully=true) {	//touching bar, aligned with goal -> aligned with tape (approx)
	int direction = (reversed ? -1 : 1);
	moveLiftToSafePos();

	//pick up side goal
	driveAndGoal((startingFromBar ? 15 : 10), OUT, false, true);
	driveStraight(startingFromBar ? 30 : 20);

	//position robot facing middle of 10pt bar
	if (fieldCones && NUM_EXTRA_CONES>0) {
		moveGoalIntake(IN, true);
		wait1Msec(500);

		driveStraight(7);

		waitForMovementToFinish(goalIntake);

		stackNewCone(true, true);
		//maybeAbort();	- TODO: abort!


		for (int i=0; i<NUM_EXTRA_CONES; i++) {	//TODO: numAutonCones
			driveStraight(7);

			while (stacking || roller.moving==DURATION) EndTimeSlice();

			if (i == 0) {
				setPower(roller, 127);
				setLiftState(L_FIELD);
				moveFourBar(false, false);
				waitForMovementToFinish(lift, INTAKE_DURATION);
			}
			else {
				stackNewCone(true, true);
			}

			stackNewCone();
		}

		driveStraight(-60 - 7 * NUM_EXTRA_CONES, true);

		while (stacking) EndTimeSlice();
		moveLiftToSafePos();

		while (driveData.isDriving) EndTimeSlice();
	}
	else {
		driveAndGoal(-42, IN);

		maybeAbort();
	}

	if (zone != FIVE) {	//ten or twenty
		if (TURN_CHEAT) {
			turn(-direction * 45, true/*, false, 40, 127, -30, 250, 20, false*/);
			long timer = resetTimer();
			while (turnData.isTurning && time(timer) < 1000) EndTimeSlice();
			turnData.isTurning = false;
		}
		else {
			turn(-direction * 45);	//turnDriveTurn?
		}

		driveStraight(middle||zone==TWENTY ? -27 : -12, true);
		while (driveData.totalDist < 5) EndTimeSlice();
		if (!(NUM_EXTRA_CONES>0 && fieldCones)) stackNewCone(false, true);
		while (driveData.isDriving) EndTimeSlice();

		turn(-direction * 90);

		while (stacking) EndTimeSlice();
		//moveLiftToSafePos();

		scoreGoal(zone==TWENTY, align, intakeFully);
	}
	else {	//score in 5pt
		stackNewCone();
		turn(direction * 200, true);
		while (stacking) EndTimeSlice();
		moveLiftToSafePos();
		moveGoalIntake(OUT);
		while (turnData.isTurning) EndTimeSlice();
		driveStraight(-15);
		if (variant) moveGoalIntake(IN, true);
	}
}

void middleGoal(bool left, bool twentyPt=true, bool middle=false, bool align=true, bool intakeFully=true) { //aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);                                                                          //if twentyPt and middle are true, assumes starting from bar (not tape)

	moveLiftToSafePos();

	moveGoalIntake(OUT, true);	//TODO: check if intake is out?
	if (twentyPt && middle) driveStraight(BAR_TO_LINE_DIST);
	waitForMovementToFinish(goalIntake);

	driveStraight(LINE_TO_GOAL_DIST);

	driveAndGoal(-29, IN);
	if (twentyPt) {	//preload
		stackNewCone();
		moveLiftToSafePos(false);
	}

	if (middle || twentyPt) {
		turn(-90 * direction);
		driveStraight(-GOAL_TO_MID_DIST - 3.5);
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

void crossFieldGoal(bool twentyPt, bool neer, bool middle=false, bool clearCones=false) {	//aligned to tape -> aligned with tape (other side)
	int nearOffset = neer ? 50 : 0;	//meant to be near. It doesn't work if you spell it correctly. I'm serious.

	moveGoalIntake(OUT, true);
	if (!neer) alignToBar(false, 1500);

	driveStraight(75 - nearOffset);

	if (twentyPt || middle) {
		driveAndGoal(40+nearOffset, IN);

		turn(-90);
		driveStraight(GOAL_TO_MID_DIST-2);
		if (clearCones) moveGoalIntake(MID);	//to push cones to side
		turn(90);
		if (clearCones) moveGoalIntake(IN);
	}
	else {
		driveAndGoal(25+nearOffset, IN);
	}

	scoreGoal(twentyPt);
}
//#endregion

//#region routines
task skillz() {
	turnDefaults.reversed = false;

	if (SKILLZ_VARIANT) {
		sideGoal();	//near left side goal to twenty

		turnDriveTurn(90, GOAL_TO_MID_DIST, 90);

		middleGoal(true, false);	//near left middle goal to ten

		//near right middle goal
		driveStraight(-LINE_TO_GOAL_DIST);
		turn(-90);
		moveGoalIntake(OUT);
		driveStraight(2 * GOAL_TO_MID_DIST);
		moveGoalIntake(IN);
		turn(90);
		driveStraight(BAR_TO_LINE_DIST + LINE_TO_GOAL_DIST);
		scoreGoal(false);
	}
	else {
		middleGoal(true, true, true);	//near left middle goal to 20Pt zone

		turnDriveTurn(-90, GOAL_TO_MID_DIST, -95);	//TODO: turnToLine() (& similar below)
		moveGoalIntake(OUT);
		//alignToBar(false, 1000);

		middleGoal(false, false, false, false);	//near right middle goal to right 10pt

		turn(-180);
	}

	crossFieldGoal(true, false);	//far right middle goal to 20pt

	//far left middle goal to...
	moveGoalIntake(OUT, true);
	turnDriveTurn(-90, GOAL_TO_MID_DIST-3, -90);
	//alignToBar(false, 1500);

	if (CROSS_FIELD_SKLZ) {	//...near left
		crossFieldGoal(false, true);

		//temp (repetitive)
		turnDriveTurn(-90, 18, -45);

		sideGoal((SKILLZ_5PT ? FIVE : TEN), false, true, true, false);	//far left side goal to left 10pt
	}
	else {	//...far left (or center?)
		middleGoal(false, false/*, !PARK_IN_SKILLS*/);	//temp

		if (PARK_IN_SKILLS) {
			turn(-140);
			driveForDuration(2000);
		}
		else {
			turnDriveTurn(-90, 18, -45);

			sideGoal((SKILLZ_5PT ? FIVE : TEN), false, true, true, false);	//far left side goal to left 10pt

			turn(160);
			moveGoalIntake(IN, true);
			driveForDuration(2000);
		}
	}

	autonDebug[0] = time(autonTimer);
}

task antiMark() {
	//if (ANTI_MARK == 1) {
		wait1Msec(DEFENSIVE_DELAY);
		driveStraight(15);
		turn(50);
		driveStraight(75);
		driveAndGoal(-40, OUT);
		turn(35);
		driveStraight(20);
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
	autonTimer = resetTimer();

	turnDefaults.reversed = sidePos < SIDE_SWITCH_POS;	//TODO: put this val in config
	variant = abs(sidePos - SIDE_SWITCH_POS) < 1500;

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else if (modePos > 1500) {	//side goal
		zoneType zone;

		if (modePos > 3560)
			zone = TWENTY;
		else if (modePos > 2225)
			zone = TEN;
		else
			zone = FIVE;

		sideGoal(zone, false, false, false, true, variant&&NUM_EXTRA_CONES>0);

		if (variant || NUM_EXTRA_CONES>0) {	//drive to other side
			switch (zone) {
				case FIVE:
					turn(-150);
					break;
				case TEN:
					turnDriveTurn(90, 13);
					break;
				case TWENTY:
					turnDriveTurn(90, 28);
					break;
			}

			driveStraight(60 * (zone==FIVE ? 1 : -1));
		}
	}
	else if (modePos > 305) {	//defensive
		if (variant)
			startTask(antiMark);
		else
			driveForDuration(2000, 127);
	}

	autonDebug[0] = time(autonTimer);
	while (true) EndTimeSlice();
}
