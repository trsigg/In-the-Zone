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

	goalIntake.moving = NO;
	currGoalState = IN;
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

void turnToAbsAngle(float angle, bool runAsTask=false, float in1=turnDefaults.rampConst1, float in2=turnDefaults.rampConst2, float in3=turnDefaults.rampConst3, float in4=turnDefaults.rampConst4, float in5=turnDefaults.rampConst5) {
	int wtf = /*fabs(*/gyroVal(drive)/*) * (angle>0 ? 1 : -1)*/;	//i am 100000% serious that it doesn't work without this. Also it will crash if you use copysign
	turn(angle + wtf, runAsTask, in1, in2, in3, in4, in5);
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
	driveStraight(dist, runAsTask, 10, 0.1, 50, 0.05, 250);
}

/*void accuTurn(int dist, bool runAsTask=false) {
	turn(dist, runAsTask, driveDefaults.rampConst1, driveDefaults.rampConst2, driveDefaults.rampConst3, 0.05, 500);
}*/

void turnToRealign() {
	if (fabs(gyroVal(drive)) > 10) {
		turnToAbsAngle(0);

		resetGyro(drive);
	} else generalDebug[1] = gyroVal(drive);
}

void aboutFace(float angle=0) {
	createManeuver(goalIntake, goalPos[MID]-250, false);
	turnToAbsAngle(angle, false, 6, 0.02, 22, 0.05, 500);
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
void scoreGoal(bool twentyPt, bool align=true, bool intakeFully=true) {	//behind 10pt bar -> aligned with tape (approx)
	if (twentyPt) {
		moveGoalIntake(OUT, true);
		driveForDuration(1000, 80, 20);	//drive over bar
		waitForMovementToFinish(goalIntake);
		//driveForDuration(250);	//push goal to back of zone
		driveForDuration(375, -127);	//TODO: remove?
		driveStraight(align ? -11 : -9);	//back out
	}
	else {
		moveGoalIntake(OUT, true);
		alignToBar();
		waitForMovementToFinish(goalIntake);
		driveStraight(-9);	//back out
	}

	moveGoalIntake(intakeFully ? IN : MID);
	//wait1Msec(200);	//while (getPosition(goalIntake) < goalPos[MID]) EndTimeSlice();

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

void sideGoal(zoneType zone=TWENTY, bool middle=false, int numExtraCones=0, bool reversed=false, bool startingFromBar=true, bool align=false, bool intakeFully=true, bool hasFirstCone=true) {	//touching bar, aligned with goal -> aligned with tape (approx)
	int direction = (reversed ? -1 : 1);
	int distAdjustment = -interConeDist[robot] * numExtraCones;

	switch (zone) {
		case FIVE:
			distAdjustment += 11;
			break;
		case TEN:
			distAdjustment -= 5;
			break;
	}

	resetGyro(drive);
	moveLiftToSafePos();

	//pick up side goal
	if (SKILLZ_MODE) {
		if (startingFromBar) {
			driveAndGoal(45, OUT, false, true);
		}
		else {
			driveAndGoal(10, OUT, false, true);
			quadDrive(19);
		}
	} else {
		if (startingFromBar) moveGoalIntake(OUT, false);
		driveStraight(startingFromBar ? 53 : 45, true);
		while ((driveData.distance - driveData.totalDist) > 12) EndTimeSlice();
		moveGoalIntake(IN, false);
	}

	//position robot facing middle of 10pt bar
	if (numExtraCones > 0) {
		maybeAbort();
		if (hasFirstCone) stackNewCone();
		turnToRealign();

		for (int i=0; i<numExtraCones; i++) {
			if (i != 0) {
				driveStraight(interConeDist[robot], true);
				stackNewCone();
			}

			while (driveData.isDriving) EndTimeSlice();

			if (stacking) goToSafe = false;

			turnToRealign();

			while (stacking) EndTimeSlice();

			//intake
			#ifdef ROLLER
				setPower(roller, 127);
			#endif

			if (goToSafe || !hasFirstCone && i==0) {
				moveFourBar(false, false);
				setLiftState(L_FIELD, true);
			}

			waitForMovementToFinish(lift, intakeDuration[robot]);
		}

		stackNewCone();

		turnToRealign();
		driveStraight(-36 + distAdjustment);
	}
	else {
		turnToRealign();

		if (SKILLZ_MODE)
			driveAndGoal(-36, IN);
		else
			driveStraight(-36 + distAdjustment);

		maybeAbort();
	}

	if (zone == FIVE) {	//score in 5pt
		if (numExtraCones == 0) stackNewCone();
		turn(direction * 180, true);
		while (stacking) EndTimeSlice();
		liftToConeSafePos();
		while (turnData.isTurning) EndTimeSlice();
		moveGoalIntake(OUT);
		quadDrive(-13);
		if (intakeFully) moveGoalIntake(IN);
	}
	else {	//ten or twenty
		if (middle || zone==TWENTY || DRIVE_FOR_10) {
			turn(direction * 135);

			if (SKILLZ_MODE)
				accuDrive(zone==TEN ? 12 : 25);
			else
				driveStraight(zone==TEN ? 3 : 21);

			turn(direction * 90);	//accu
		}
		else {	//near ten
			turn(direction * 205);	//225
		}

		while (stacking) EndTimeSlice();
		liftToConeSafePos();

		scoreGoal(zone==TWENTY, align, intakeFully);
	}
}

void middleGoal(bool left, bool twentyPt=true, bool middle=false, bool align=false, bool intakeFully=false) { //aligned with goal -> aligned with tape (approx)
	int direction = (left ? 1 : -1);                                                                          //if twentyPt and middle are true, assumes starting from bar (not tape)

	moveLiftToSafePos();

	if (getPosition(goalIntake) < goalPos[OUT]) moveGoalIntake(OUT);
	driveStraight(barToLineDist[robot] + lineToGoalDist[robot]);

	driveAndGoal(-20, IN, true/*, true*/);

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
		turn(-180);
		//driveForDuration(1250, 40);	//align to 10pt bar
	}

	resetGyro(drive);

	while (stacking) EndTimeSlice();

	moveLiftToSafePos();
	scoreGoal(twentyPt, false, intakeFully);

	if (align) {
		alignToBar(true, (twentyPt ? 600 : 1200));
		driveStraight(-barToLineDist[robot]);
	}
}

void crossFieldGoal(bool twentyPt, bool neer, bool intakeFully=false, bool middle=false, bool clearCones=false, bool align=false) {	//aligned to tape -> aligned with tape (other side)
	int nearOffset = neer ? 50 : 0;	//meant to be near. It doesn't work if you spell it correctly. I'm serious.

	moveGoalIntake(OUT/*, true*/);
	if (RECKON_IN_SKILLZ) {
		driveForDuration(1400, -50, 0);	//alignToBar(false, 1500);
		wait1Msec(250);
	}

	driveStraight(70 - nearOffset - (RECKON_IN_SKILLZ ? 0 : 15));

	if (twentyPt || middle) {
		driveAndGoal(40+nearOffset, MID);

		turn(-90);
		driveStraight(goalToMidDist[robot]-2);
		if (clearCones) moveGoalIntake(MID);	//to push cones to side
		turn(90);
		if (clearCones) moveGoalIntake(IN);
	}
	else {
		driveAndGoal(31+nearOffset, MID);
	}

	scoreGoal(twentyPt, align, intakeFully);
}

void backUpGoal(bool redSide, bool intakeFully=false, bool reversed=false) {
	int direction = (reversed ? -1 : 1);

	moveGoalIntake(MID, true);
	driveStraight(-lineToGoalDist[robot] - (redSide ? 13 : 15.5));	//accuDrive?
	waitForMovementToFinish(goalIntake);
	if (redSide)
		turnToAbsAngle(-direction * 93);
	else
		turn(-direction * 90);
	//turn(-direction * (redSide ? 90 : 90));	//accu
	moveGoalIntake(OUT);
	driveStraight(redSide ? 22 : 22);
	driveAndGoal(redSide ? 7 : 4, IN);
	turn(direction * (redSide ? 100 : 95));	//accu
	driveStraight(barToLineDist[robot] + lineToGoalDist[robot] + 8);
	scoreGoal(false, false, intakeFully);
}
//#endregion

//#region routines
void skillz() {
	sideGoal(TWENTY, false, 0, false, true, false, false);	//near left side goal to twenty

	moveGoalIntake(OUT, true);
	turnDriveTurn(90, goalToMidDist[robot], 90);	//near left middle goal to ten

	middleGoal(true, false);

	backUpGoal(true); //near right middle goal
	aboutFace(-80);

	crossFieldGoal(false, false);	//far right middle goal to 10pt

	//far left middle goal to far right
	backUpGoal(false);

	//far left side goal to middle ten
	turnToAbsAngle(-20);
	driveAndGoal(45, OUT);
	driveAndGoal(-62, IN);
	turn(120);
	//driveStraight(-12);
	//turn(90);
	scoreGoal(true, false);

	//far right side goal to middle ten
	turnDriveTurn(90, 20, 45);
	driveStraight(60);
	//sideGoal(TEN, true, 0, false, false, false, true);
}

void altSkillz() {
	sideGoal(TWENTY, false, 0, true, true, false, false);

	turnDriveTurn(-90, goalToMidDist[robot], -85);
	middleGoal(false, false, false, false, false);

	aboutFace(10);
	crossFieldGoal(false, false, true);

	turnDriveTurn(90, 10, 45);
	sideGoal(TWENTY, false, 0, false, false, false, false);

	turnDriveTurn(-90, goalToMidDist[robot], -90);
	middleGoal(false, false, false, false, false);

	aboutFace(10);
	crossFieldGoal(false, false, true);

	turnDriveTurn(45, 10, 45);
	sideGoal(TEN, true, 0, false, false, false, false);

	turn(20);
	driveForDuration(1500, -127, 0); //driveStraight(-60);
}

void counterDefensive(int defensiveDelay) {
	wait1Msec(defensiveDelay);
	driveStraight(10);
	turn(47);
	driveStraight(75);
	driveAndGoal(-40, OUT, false, false, 2000);
	turn(30);
	driveStraight(20);
	driveAndGoal(-30, IN, true);
	turn(130);
	driveStraight(35);
	scoreGoal(false);
}

void stationaryScore() {
	//moveFourBar(true);
	setLiftState(S_BASE_POS);

	driveStraight(20);

	waitForLiftingToFinish();
	moveFourBar(false, false);
	setIntakeState(false);

	moveFourBar(true, false);

	/*setLiftTargetAndPID(liftPos[S_BASE_POS] + 200);
	waitForLiftingToFinish(outtakeDuration[robot]);*/

	setToStillSpeed(roller);
	//moveFourBar(true, true);
	createManeuver(goalIntake, goalPos[MID]-100);
	setLiftTargetAndPID(liftPos[L_SAFE] + 50);
	driveStraight(-16);
	waitForLiftingToFinish();
}
//#endregion

task selfDestruct() {
	wait1Msec(SKILLZ_MODE ? 60000 : 15000);
	//stopAllTasks();
	stopTask(autonomous);
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

	int sidePos   = SensorValue[ sidePot[robot] ];
	int modePos   = SensorValue[ modePot[robot] ];
	int configPos = SensorValue[ configPot[robot] ];
	int extraCones;
	autonTimer = resetTimer();

	turnDefaults.reversed = sidePos > sideSwitchPos[robot];	//TODO: put this val in config
	variant = abs(sidePos - sideSwitchPos[robot]) < 1600;

	if (configPos < 370)
		extraCones = 0;
	else if (configPos < 1915)
		extraCones = 1;
	else if (configPos < 3760)
		extraCones = 2;
	else
		extraCones = 3;

	if (SKILLZ_MODE) {
		turnDefaults.reversed = false;

		if (SKILLZ_VARIANT)
			altSkillz();
		else
			skillz();
	}
	else if (modePos < 2520) {	//side goal
		zoneType zone;

		if (modePos < 530)
			zone = TWENTY;
		else if (modePos < 1620)
			zone = TEN;
		else
			zone = FIVE;

		if (!(STACK_SIDE_CONES && variant)) extraCones = 0;

		sideGoal(zone, false, extraCones, false, true, false, (zone!=FIVE || variant || STACK_SIDE_CONES));

		//if (variant || extraCones>0) {	//drive to other side
			switch (zone) {
				case FIVE:
					turn(45);
					break;
				case TEN:
					//turnDriveTurn(90, 3);
					turn(-10);
					break;
				case TWENTY:
					turnDriveTurn(90, 21);
					break;
			}

			driveStraight(-75);
		//}
	}
	else if (modePos < 3820) {	//defensive
		int defensiveDelay;

		if (configPos < 370)
			defensiveDelay = 0;
		else if (configPos < 1915)
			defensiveDelay = 500;
		else if (configPos < 3760)
			defensiveDelay = 1000;
		else
			defensiveDelay = 2000;

		counterDefensive(defensiveDelay);
	} else if (variant) {
		//driveForDuration(2000, 127);
		stationaryScore();

		turn(-86);

		sideGoal(TEN, false, extraCones, false, false, false, true, false);
		//sideGoal(NTY, middl, extraCones, rever, omBar, align, inta, hasFi);
	}

	autonDebug[0] = time(autonTimer);
	while (true) EndTimeSlice();
}
