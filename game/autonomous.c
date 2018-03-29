#include "autostacking.c"
#include "mobileGoal.c"
#include "testing.c"

#ifdef ROLLER
	#include "rollers.c"
#endif


enum zoneType { FIVE, TEN, TWENTY };

int autonDebug[2];	// { time of auton routine, follower value on abort }
int autonTimer;
bool variant = false;
int numRetries = 0;

task autonUpdateTask() {
	while (true) {
		executeManeuvers();
		updateMotorConfig();
		logData();
		EndTimeSlice();
	}
}

//#region preparation
void prepareForAuton() {
	resetLiftEncoders();
	stopLiftTargeting();
	startAutoStacking();
	numCones = 0;

	if (!USE_ENC_CORR) {
		driveDefaults.kP_c = 0;
		driveDefaults.kI_c = 0;
		driveDefaults.kD_c = 0;
	}

	#ifdef PNEUMATIC
		setState(intake, true);
		setState(fourBar, false);
	#else
		setPower(fourBar, -fbStillSpeed[robot]);
	#endif

	#ifdef ROLLER
		setToStillSpeed(roller);
	#endif
}
//#endregion

//#region conditional abort
void stopAllMotors() {	//TODO: reposition?
	for (tMotor port=port1; port<=port10; port++) {
		motor[port] = 0;
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
			if (SensorValue[leftLine[robot]] < l_lineThresh[robot]) {
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
			if (SensorValue[rightLine[robot]] < r_lineThresh[robot]) {
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
		while (SensorValue[leftLine[robot]] < l_lineThresh && SensorValue[rightLine[robot]] < r_lineThresh[robot]) EndTimeSlice();
	else
		while (SensorValue[backLine[robot]] < b_lineThresh[robot]) EndTimeSlice();

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
	while (SensorValue[frontSonar[robot]] > dist || SensorValue[frontSonar[robot]] == -1) EndTimeSlice();	//TODO: use drive sonar & sonarFartherThan()
	setDrivePower(drive, -brakePower, -brakePower);
	wait1Msec(brakeDuration);
	setDrivePower(drive, 0, 0);
}

void driveAndSonar(int driveDist, int sonarDist, int power=30, int brakePower=10, int brakeDuration=150) {
	driveStraight(driveDist);
	driveToSonarDist(sonarDist, power, brakePower, brakeDuration);
}

void driveForDuration(int duration, int beginPower=127, int endPower=0) {
	long timer = resetTimer();

	while (time(timer) < duration) {
		setDrivePower(drive, beginPower, beginPower);
		EndTimeSlice();
	}

	setDrivePower(drive, endPower, endPower);
}

void alignToBar(bool forward=true, int duration=600) {
	int direction = (forward ? 1 : -1);
	driveForDuration(duration, 35*direction, 35*direction);
}

void turnDriveTurn(int angle, int dist, int angle2=0) {
	turn(angle);
	driveStraight(dist);
	turn(angle2==0 ? -angle : angle2);
}

void quadDrive(int dist, bool runAsTask=false) {
	driveStraight(dist, runAsTask, 30, 120, -20, 250, 20, false);
}

void accuDrive(int dist, bool runAsTask=false) {
	driveStraight(dist, runAsTask, 10, 0.1, 50);
}

void driveAndGoal(int dist, goalState state, bool stackCone=false, bool quadRamp=false, int intakeDelay=500) {
	moveLiftToSafePos();
	moveGoalIntake(state, true);

	if (state!=OUT && dist<0)
		wait1Msec(intakeDelay);

	if (quadRamp)
		quadDrive(dist, true);
	else
		driveStraight(dist, true);

	waitForMovementToFinish(goalIntake);

	if (stackCone) stackNewCone();

	while (driveData.isDriving /*|| (stacking && stackCone)*/) EndTimeSlice();
}
//#endregion

//#region routine portions
void scoreGoal(bool twentyPt=true, bool align=true, bool intakeFully=true) {	//behind 10pt bar -> aligned with tape (approx)
	if (twentyPt) {
		moveGoalIntake(OUT, true);
		driveForDuration(1000, 90, 20);	//drive over bar
		waitForMovementToFinish(goalIntake);
		//driveForDuration(250);	//push goal to back of zone
		driveForDuration(375, -127);
	}
	else {
		moveGoalIntake(OUT, true);
		alignToBar();
		waitForMovementToFinish(goalIntake);
		driveStraight(-9);	//back out
	}

	moveGoalIntake(intakeFully ? IN : MID);
	//wait1Msec(200);	//while (getPosition(goalIntake) < goalPos[MID]) EndTimeSlice();

	if (twentyPt) {
		driveStraight(align ? -11 : -15);	//back out
		/*if (align)
			driveStraight(-15);
		else
			driveForDuration(75, -127)*/
	}

	if (align) {
		alignToBar(true, (twentyPt ? 700 : 600));
		driveStraight(-barToLineDist[robot]);
	}

	//waitForMovementToFinish(goalIntake);	//if something breaks with the goal intake, uncomment this line
	if (isMobileGoalLoaded() && numRetries<MAX_GOAL_RETRIES && RETRY_GOAL_FAILS) {
		numRetries++;
		playSound(soundBeepBeep);
		autonDebug[1] = SensorValue[ goalLine[robot] ];
		scoreGoal(twentyPt, align, intakeFully);
	}
	else {
		numCones = 0;
		numRetries = 0;
	}
}

void sideGoal(zoneType zone=TWENTY, bool middle=false, int numExtraCones=0, bool reversed=false, bool startingFromBar=true, bool align=false, bool intakeFully=true) {	//touching bar, aligned with goal -> aligned with tape (approx)
	int direction = (reversed ? -1 : 1);
	int distAdjustment = (zone==FIVE ? 8 : 0) - interConeDist[robot] * numExtraCones;
	moveLiftToSafePos();

	//pick up side goal
	if (SKILLZ_MODE) {
		driveAndGoal(43, OUT);
	}
	else {
		driveAndGoal((startingFromBar ? 23 : 10), OUT, false, true);

		quadDrive(startingFromBar ? 19 : 20);
	}

	//position robot facing middle of 10pt bar
	if (numExtraCones > 0) {
		for (int i=0; i<numExtraCones; i++) {
			driveStraight(interConeDist[robot], true);

			if (i == 0) {
				moveGoalIntake(IN);
				maybeAbort();
			}

			stackNewCone();

			while (driveData.isDriving) EndTimeSlice();

			while (stacking) {
				goToSafe = false;
				EndTimeSlice();
			}

			//intake
			#ifdef ROLLER
				setPower(roller, 127);
			#endif

			if (goToSafe) {
				moveFourBar(false, false);
				setLiftState(L_FIELD, true);
			}

			waitForMovementToFinish(lift);
			wait1Msec(intakeDuration[robot]);
		}

		stackNewCone();

		driveStraight(-42 + distAdjustment);

		//while (stacking) EndTimeSlice();
	}
	else {
		driveAndGoal(-42 + distAdjustment, IN);

		maybeAbort();
	}

	if (zone == FIVE) {	//score in 5pt
		if (numExtraCones == 0) stackNewCone();
		turn(-direction * 160, true);
		while (stacking) EndTimeSlice();
		liftToConeSafePos();
		while (turnData.isTurning) EndTimeSlice();
		moveGoalIntake(OUT);
		driveStraight(-10);
		if (intakeFully) moveGoalIntake(IN);
	}
	else {	//ten or twenty
		if (TURN_CHEAT) {
			turn(-direction * 45, true/*, false, 40, 127, -30, 250, 20, false*/);
			long timer = resetTimer();
			while (turnData.isTurning && time(timer) < 1000) EndTimeSlice();
			turnData.isTurning = false;
		}
		else {
			turn(-direction * 45);	//turnDriveTurn?
		}

		driveStraight(middle||zone==TWENTY ? -22 : -7, true);
		while (driveData.totalDist < 5) EndTimeSlice();
		if (numExtraCones == 0) stackNewCone();
		while (driveData.isDriving) EndTimeSlice();

		turn(-direction * 90);

		while (stacking) EndTimeSlice();
		liftToConeSafePos();

		scoreGoal(zone==TWENTY, align, intakeFully);
	}
}

void middleGoal(bool left, bool twentyPt=true, bool middle=false, bool align=false, bool intakeFully=false) { //aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);                                                                          //if twentyPt and middle are true, assumes starting from bar (not tape)

	moveLiftToSafePos();

	moveGoalIntake(OUT, true);	//TODO: check if intake is out?
	if (twentyPt && middle) driveStraight(barToLineDist[robot]);
	waitForMovementToFinish(goalIntake);

	driveStraight(lineToGoalDist[robot] + 9);

	driveAndGoal(-25, IN, false, true);

	if (twentyPt) {	//preload
		stackNewCone();
		moveLiftToSafePos(false);
	}

	if (middle || twentyPt) {
		turn(-90 * direction);
		driveStraight(-goalToMidDist[robot] - 3.5);
		turn(-90 * direction);
	}
	else {
		turn(180);
		//driveForDuration(1250, 40);	//align to 10pt bar
	}

	while (stacking) EndTimeSlice();

	moveLiftToSafePos();
	scoreGoal(twentyPt, false, intakeFully);

	if (align) {
		alignToBar(true, (twentyPt ? 600 : 1200));
		driveStraight(-barToLineDist[robot]);
	}
}

void crossFieldGoal(bool twentyPt, bool neer, bool middle=false, bool clearCones=false, bool align=false) {	//aligned to tape -> aligned with tape (other side)
	int nearOffset = neer ? 50 : 0;	//meant to be near. It doesn't work if you spell it correctly. I'm serious.

	moveGoalIntake(OUT/*, true*/);
	if (!neer) driveForDuration(500, -45, 0);	//alignToBar(false, 1500);
	waitForMovementToFinish(goalIntake);
	wait1Msec(50);

	driveStraight(75 - nearOffset);

	if (twentyPt || middle) {
		driveAndGoal(40+nearOffset, IN);

		turn(-90);
		driveStraight(goalToMidDist[robot]-2);
		if (clearCones) moveGoalIntake(MID);	//to push cones to side
		turn(90);
		if (clearCones) moveGoalIntake(IN);
	}
	else {
		driveAndGoal(31+nearOffset, IN);
	}

	scoreGoal(twentyPt, align, false);
}

void backUpGoal(bool startingFromBar=false, bool intakeFully=true, bool reversed=false) {
	int direction = (reversed ? -1 : 1);

	moveGoalIntake(IN, true);
	accuDrive(-lineToGoalDist[robot] - (startingFromBar ? 8 : 6));
	waitForMovementToFinish(goalIntake);
	turn(-direction * (startingFromBar ? 85 : 90));
	moveGoalIntake(OUT);
	driveStraight(startingFromBar ? 21 : 27);
	driveAndGoal(startingFromBar ? 10 : 8, IN);
	turn(direction * (startingFromBar ? 90 : 95));
	driveStraight(barToLineDist[robot] + lineToGoalDist[robot]);
	scoreGoal(false, false, intakeFully);
}
//#endregion

//#region routines
task skillz() {
	turnDefaults.reversed = false;

	if (SKILLZ_VARIANT) {
		sideGoal(TWENTY, false, 0, false, true, true, false);
		//sideGoal();	//near left side goal to twenty

		turnDriveTurn(90, goalToMidDist[robot], 90);

		middleGoal(true, false);	//near left middle goal to ten

		backUpGoal(); //near right middle goal
	}
	else {
		middleGoal(true, true, true);	//near left middle goal to 20Pt zone

		turnDriveTurn(-90, goalToMidDist[robot]-0.5, -90);	//TODO: turnToLine() (& similar below)
		moveGoalIntake(OUT);
		//alignToBar(false, 1000);

		middleGoal(false, false, false, false);	//near right middle goal to right 10pt
	}

	turn(-175);

	crossFieldGoal(CROSS_FIELD_SKLZ, false);	//far right middle goal to 10pt

	//far left middle goal to...
	if (!SKILLZ_VARIANT) {
		turnDriveTurn(-90, 2*goalToMidDist[robot]-5, -90);
		moveGoalIntake(OUT);
		//alignToBar(false, 1500);
	}

	if (CROSS_FIELD_SKLZ) {	//...near left
		crossFieldGoal(false, true);

		//temp (repetitive)
		turnDriveTurn(-90, 18, -45);

		sideGoal((SKILLZ_5PT ? FIVE : TEN), false, 0, true, false);	//far left side goal to left 10pt
	}
	else {	//...far left (or center?)
		if (SKILLZ_VARIANT) {
			backUpGoal(true, false);

			turn(-125);
			//sideGoal(TWENTY, false, 0, true);
			/*driveAndGoal(20, OUT);
			quadDrive(30);*/
			driveAndGoal(50, OUT);
			driveAndGoal(-50, IN);
			turn(30);
			driveStraight(-19);
			turn(90);
			scoreGoal(true);
		}
		else {
			middleGoal(false, true/*, !PARK_IN_SKILLS*/);	//temp
		}

		if (PARK_IN_SKILLS) {
			turn(-140);
			driveForDuration(2000);
		}
		else {
			turnDriveTurn(-90, 18, -45);

			sideGoal((SKILLZ_5PT ? FIVE : TEN), false, 0, true, false);	//far left side goal to left 10pt

			turn(160);
			moveGoalIntake(IN, true);
			driveForDuration(2000);
		}
	}

	autonDebug[0] = time(autonTimer);
}

task antiMark() {
	if (ANTI_MARK == 1) {
		wait1Msec(DEFENSIVE_DELAY);
		driveStraight(13);
		turn(45);
		driveStraight(75);
		driveAndGoal(-40, OUT, false, false, 1250);
		turn(30);
		driveStraight(20);
		moveGoalIntake(IN, false);
		stackNewCone(false, false);
	}
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
	}
}
//#endregion

task selfDestruct() {
	wait1Msec(SKILLZ_MODE ? 60000 : 15000);
	//stopAllTasks();
	stopTask(autonomous);
	stopTask(skillz);
	stopTask(antiMark);
	stopAllMotors();
}

#ifdef RUN_AUTON_AS_MAIN
task main() {
#else
task autonomous() {
#endif
	prepareForAuton();
	handleTesting();
	startTask(autonUpdateTask);

	if (ABORT_AFTER_15) startTask(selfDestruct);

	int sidePos = SensorValue[ sidePot[robot] ];
	int modePos = SensorValue[ modePot[robot] ];
	autonTimer = resetTimer();

	turnDefaults.reversed = sidePos > sideSwitchPos[robot];	//TODO: put this val in config
	variant = abs(sidePos - sideSwitchPos[robot]) < 1600;

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else if (modePos < 2520) {	//side goal
		zoneType zone;
		int extraCones;

		if (modePos < 530) {
			zone = TWENTY;
			extraCones = 2;
		}
		else if (modePos < 1620) {
			zone = TEN;
			extraCones = 2;
		}
		else {
			zone = FIVE;
			extraCones = 2;
		}

		if (!(STACK_SIDE_CONES && variant)) extraCones = 0;

		sideGoal(zone, false, extraCones, false, true, false, (zone!=FIVE || variant || STACK_SIDE_CONES));

		//if (variant || extraCones>0) {	//drive to other side
			switch (zone) {
				case FIVE:
					turn(35);
					break;
				case TEN:
					turnDriveTurn(90, 7);
					break;
				case TWENTY:
					turnDriveTurn(90, 21);
					break;
			}

			driveStraight(-75);
		//}
	}
	else if (modePos < 3820) {	//defensive
		if (variant)
			startTask(antiMark);
		else
			driveForDuration(2000, 127);
	}

	autonDebug[0] = time(autonTimer);
	while (true) EndTimeSlice();
}
