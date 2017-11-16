#pragma config(Sensor, in1,    sidePot,        sensorPotentiometer)
#pragma config(Sensor, in2,    liftSensor,     sensorPotentiometer)
#pragma config(Sensor, in4,    leftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    backLine,       sensorLineFollower)
#pragma config(Sensor, in6,    rightLine,      sensorLineFollower)
#pragma config(Sensor, in7,    hyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  chainSensor,    sensorQuadEncoder)
#pragma config(Motor,  port1,           RDrive1,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           RDrive2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           chain1,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           intake,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           LDrive1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           lift2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           chain2,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           goal1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          goal2,         tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config - TODO: change testing parameter scheme
#define HOLD_LAST_CONE true
#define L_USING_ENC    false
#define CH_USING_ENC   true
#define SKILLZ_MODE    false
#define MULTIPLE_PIDs  false	//if chain bar and lift use different PID consts for movement in different locations or directions
	//#subregion testing
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
int debugParameters[] = { 0, -1, 7, -1 };	//{ liftDebugStartCol, chainDebugStartCol, liftSensorCol, chainSensorCol }
	//#endsubregion
//#endregion

//#region positions
enum chainState  { CH_FIELD, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 146,      146,     42,    0,      42,   197 };

enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
int liftPos[] = { 1425,  1430,    1880,   1420,       1880,   1880,   2960 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\pd_autoMove.c"
#include "..\lib\buttonTracker.c"
#include "..\audio\downloadSounds.c"
//#endregion

//#region buttons
#define abortManeuversBtn Btn7L
#define shiftBtn					Btn7R
#define sayConeNumberBtn  Btn8L	//with shift

	//#subregion goal intake
#define goalIntakeBtn			Btn7D
#define goalOuttakeBtn		Btn7U
	//#endsubregion

		//#subsubregion cone intake
#define coneIntakeBtn     Btn6U	//intakes alone, outtakes with shift
		//#endsubsubregion

	//#subregion autopositioning
#define chainDefBtn				Btn8D	//takes chain bar to default position
#define chainStackBtn			Btn8L //takes chain bar to STACK
                                //when pressed together, they take lift to L_DEF and chain bar to default position for mode
//#define maxPosBtn         Btn7U //takes lift to L_MAX
	//#endsubregion

	//#subregion autostacking control
#define stackBtn					Btn6D
#define toggleFieldingBtn Btn7D
		//#subsubregion cone count adjustment
#define resetBtn					Btn8R	//with shift
#define increaseConesBtn	Btn8U	//with shift
#define decreaseConesBtn	Btn8D	//with shift
		//#endsubsubregion
	//#endsubregion

//#subregion manual control
#define resetEncodersBtn  Btn7U
		//#subsubregion lift
#define f_liftUpBtn				Btn8U	//fielding mode
#define f_liftDownBtn			Btn8R
#define d_liftUpBtn       Btn5U	//driver load mode
#define d_liftDownBtn     Btn5D
		//#endsubsubregion
		//#subsubregion chain bar
#define f_chainInBtn			Btn5U	//fielding mode
#define f_chainOutBtn			Btn5D
#define d_chainInBtn			Btn8R	//driver load mode
#define d_chainOutBtn			Btn8U
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT   880.1    //conversion factor between radians and potentiometer values
#define L_GEAR_RATIO 5	//gear ratio between lift bar angle and sensors
#define RAD_TO_ENC   (L_GEAR_RATIO * 180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT = (L_USING_ENC ? RAD_TO_ENC : RAD_TO_POT);
const float L_CORR_FCTR = (L_USING_ENC ? RAD_TO_POT/RAD_TO_ENC : 1);
const float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_LIFT);	//used in autostacking
const float CH_CORR_FCTR = (CH_USING_ENC ? RAD_TO_POT/RAD_TO_ENC : 1);
#define R_LINE_THRESHOLD 2960
#define L_LINE_THRESHOLD 3060
#define B_LINE_THRESHOLD 2870
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 2.75
#define LIFT_LEN    14.0
#define LIFT_OFFSET 2.0
#define GOAL_TO_MID_DIST 17.5
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 15
#define LIFT_STILL_SPEED   15
#define L_AUTO_SS_MARGIN   50
#define CHAIN_STILL_SPEED  15
#define CH_AUTO_SS_MARGIN	 0
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES       0 //number of cones for which lift does not move
#define RECKLESS_CONES     1 //number of cones for which chain bar goes directly to STACK (not CH_SAFE first)
#define NO_OFFSET_CONES    1 //number of cones for which the lift goes straight to liftAngle2
#define D_LIFT_EARLY_CONES 3 //number of cones in driver load mode for which lift and chain go to defaults simultaneously
#define MAX_NUM_CONES      15
	//#endsubregion
	//#subregion timing
#define INTAKE_DURATION  400	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 450
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1750
	//#endsubregion
//#endregion

//#region timers
#define GOAL_TIMER T1
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
bool fielding = true;	//whether robot is intaking cones from the driver load or field
int goalDirection = 0;	//0: not moving; -1: intaking; 1: outtaking
float liftAngle1, liftAngle2;	//the target angles of lift sections during a stack maneuver

	//#subregion autopositioning
enum AutoPosState { NO_POS, CHAIN_DEF, CH_STACK, FULL_DEF, MAX };
AutoPosState posState = NO_POS;
	//#endsubregion

motorGroup lift;
motorGroup chainBar;
motorGroup goalIntake;
motorGroup coneIntake;
//#endregion

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

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeAutoMovement();

	//configure drive
	initializeDrive(drive, true);
	setDriveMotors(drive, 4, LDrive1, LDrive1, RDrive1, RDrive2);
	attachEncoder(drive, leftEnc, LEFT, true);
	attachEncoder(drive, rightEnc, RIGHT, true, 4.0, 2.0);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 2, lift1, lift2);
	initializeTargetingPID(lift, 0.4*L_CORR_FCTR, 0.005*L_CORR_FCTR, 5*L_CORR_FCTR, 10);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	addSensor(lift, liftSensor);
	if (L_USING_ENC) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//configure chain bar
	initializeGroup(chainBar, 2, chain1, chain2);
	initializeTargetingPID(chainBar, 7, 0.01, 20, 10);	//gain setup in setChainBarPIDmode (TODO: fctr)
	addSensor(chainBar, chainSensor);
	if (CH_USING_ENC) configureEncoderCorrection(chainBar, chainPos[CH_MAX]);

	setLiftControlMode(true);	//configures fielding control for chain bar and lift

	//configure mobile goal intake
	initializeGroup(goalIntake, 2, goal1, goal2);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);

	//configure cone intake
	initializeGroup(coneIntake, 1, intake);

	//download sound files
	downloadSounds();
}

//#region audio
void speakNum(int num) {
	string fileName;
	stringFormat(fileName, "%d.wav", num);
	playSoundFile(fileName);
}
//#endregion

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

//#region chain bar
void setChainBarPIDmode(bool low) {	//	low should be true for targets below VERT
	if (low)
		initializeTargetingPID(chainBar, 2.5, 0.01, 7, 10);	//TODO: fctr
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
//#endregion

void resetEncoders() {
	resetEncoder(lift);
	resetEncoder(chainBar);
}

//#region autostacking
void waitForMovementToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=100, float chainMargin=20, float liftMargin=200) {	//TODO: fctr
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(chainBar, chainMargin) && waitForChain ||
				!errorLessThan(lift, liftMargin) && waitForLift)
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

int adjustedNumCones() {
	return limit(numCones-APATHY_CONES, 0, MAX_NUM_CONES-APATHY_CONES);
}

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_LIFT * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_MIN], liftPos[L_MAX]);
}

void stackNewCone(bool wait=false) {
	float stackHeight = CONE_HEIGHT * adjustedNumCones();

	liftAngle1 = calcLiftTargetForHeight(stackHeight + LIFT_OFFSET);
	liftAngle2 = calcLiftTargetForHeight(stackHeight);
	stacking = true;

	if (wait)
		while (stacking)
			EndTimeSlice();
}

void handleEncoderCorrection() {
	if (L_USING_ENC)
		correctEncVal(lift);

	if (CH_USING_ENC)
		correctEncVal(chainBar);
}

void executeLiftManeuvers(bool autoStillSpeed=true) {
	handleEncoderCorrection();

	if (autoStillSpeed && errorLessThan(chainBar, CH_AUTO_SS_MARGIN/CH_CORR_FCTR) && chainBar.activelyMaintining)
		setPower(chainBar, CHAIN_STILL_SPEED * (chainBar.posPID.target<=chainPos[VERT] ? 1 : -1));
	else
		maintainTargetPos(chainBar, debugParameters[1]);

	if (autoStillSpeed && errorLessThan(lift, L_AUTO_SS_MARGIN/L_CORR_FCTR) && lift.activelyMaintining && lift.posPID.target<=liftPos[L_FIELD])
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

void expelCone() {	//should be called after stacking cone
	setPower(coneIntake, -127);
	setLiftTargetAndPID(liftAngle1, false);
	waitForMovementToFinish(true, false, OUTTAKE_DURATION);
}

task autoStacking() {
	bool useOffset;
	bool liftEarly;

	while (true) {
		while (!stacking) EndTimeSlice();

		useOffset = (numCones >= NO_OFFSET_CONES);
		liftEarly = !fielding && (numCones < D_LIFT_EARLY_CONES);

		//intake cone
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(numCones<RECKLESS_CONES ? STACK : CH_SAFE);
		setLiftTargetAndPID(useOffset ? liftAngle1 : liftAngle2);

		while (!errorLessThan(lift, 200/L_CORR_FCTR)) EndTimeSlice();
		if (numCones >= RECKLESS_CONES) setChainBarState(STACK);
		waitForMovementToFinish(false);
		if (useOffset) setLiftTargetAndPID(liftAngle2/*, false*/);

		waitForMovementToFinish(true, true, 250);

		if (numCones < MAX_NUM_CONES-1) {
			expelCone();
			stacking = false;
			numCones++;

			if (liftEarly) setLiftState(L_DEF);
			setChainBarState(CH_DEF);
			setPower(coneIntake, 0);

			//return to ready positions
			while (getPosition(chainBar) < chainPos[CH_SAFE]-(fielding ? 0 : 100/CH_CORR_FCTR)) EndTimeSlice();
			if (!liftEarly) setLiftState(L_DEF);
		}
		else {
			if (!HOLD_LAST_CONE)
				expelCone();
			stacking = false;
			numCones++;

			lift.activelyMaintining = false;	//allows lift to fall down on stack
			lift.stillSpeedReversed = true;
		}

		speakNum(numCones);
	}
}
//#endregion

//#region testing
#define NUM_TARGETS 6
int targets[NUM_TARGETS] = { 0, 0, 0, 0, 1, 1 };	//chain bar, lift, driveStraight, turn, chain PID mode(low=0), lift PID mode (up=1)
bool abort = false;
bool end = false;

void logSensorVals() {
	if (debugParameters[2] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[2], getPosition(lift));
	else if (debugParameters[3] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[3], getPosition(chainBar));
}

void handlePIDinput(int index) {
	int input = targets[index];

	switch (index) {
		case 0:
			setTargetPosition(chainBar, input);
			break;
		case 1:
			setTargetPosition(lift, input);
			break;
		case 2:
			driveStraight(input);
			playSound(soundLowBuzz);
			break;
		case 3:
			turn(input);
			playSound(soundLowBuzz);
		case 4:
			setChainBarPIDmode(input == 0);
			break;
		case 5:
			setLiftPIDmode(input == 1);
			break;
	}
}

void testPIDs() {
	int prevTargets[NUM_TARGETS] = { 0, 0, 0, 0, 0, 0 };
	//arrayCopy(targets, prevTargets, NUM_TARGETS);

	while (!end) {
		for (int i=0; i<NUM_TARGETS; i++) {
			if (prevTargets[i] != targets[i]) {
				prevTargets[i] = targets[i];
				handlePIDinput(i);
			}
		}

		logSensorVals();

		if (abort) {
			stopLiftTargeting();
			setPower(lift, 0);
			setPower(chainBar, 0);

			driveData.isDriving = false;
			turnData.isTurning = false;
			setDrivePower(drive, 0, 0);

			wait1Msec(50);
			abort = false;
		}

		executeLiftManeuvers(TESTING == 1);
	}
}

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
}
//#endregion

//#region autonomous
void alignToLine(int power=60, int brakePower=10, int brakeDuration=250) {	//brakepower is absolute value (sign automatically determined)
	long leftTimer, rightTimer;
	setDrivePower(drive, power, power);

	int leftProgress=0, rightProgress=0;	//0 - moving
	                                      //1 - braking
	                                      //2 - finished

	while (leftProgress<2 || rightProgress<2) {
		if (leftProgress == 0) {
			if (SensorValue[leftLine] < L_LINE_THRESHOLD) {
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
			if (SensorValue[rightLine] < R_LINE_THRESHOLD) {
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
		while (SensorValue[leftLine]<L_LINE_THRESHOLD && SensorValue[rightLine]<R_LINE_THRESHOLD) EndTimeSlice();
	else
		while (SensorValue[backLine] < B_LINE_THRESHOLD) EndTimeSlice();

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

task moveGoalIntakeTask() {
	while (time1(GOAL_TIMER) < (goalDirection==1 ? GOAL_OUTTAKE_DURATION : GOAL_INTAKE_DURATION)) EndTimeSlice();
	setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
	goalDirection = 0;
}

void moveGoalIntake(bool in, bool runAsTask=false) {
	goalDirection = in ? -1 : 1;
	setPower(goalIntake, 127 * goalDirection);

	if (runAsTask) {
		clearTimer(GOAL_TIMER);
		startTask(moveGoalIntakeTask);
	}
	else {
		wait1Msec(in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION);
		setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
		goalDirection = 0;
	}
}

void moveLiftToSafePos(bool wait=true) {
	setChainBarState(STACK);
	setLiftTargetAndPID(liftPos[L_SAFE] + 100/L_CORR_FCTR);

	if (wait)
		while (getPosition(lift) < liftPos[L_SAFE])
			EndTimeSlice();
}

void scoreGoal(bool retract=true, bool twentyPt=true) {	//assumes robot is lined up to first bar (TODO: make retract 2nd arg)
	if (twentyPt)
		driveForDuration(1000, 127, 20);
	else
		driveForDuration(750, 60, 15);

	moveGoalIntake(false);	//extend goal intake

	driveForDuration(250);	//push goal to back of zone
	driveStraight(twentyPt ? -17 : -7);

	numCones = 0;
	if (retract) moveGoalIntake(true);	//retract goal intake
}

void driveAndGoal(int dist, bool in, bool stackCone=false, bool pulseIntake=false, bool quadRamp=false, int intakeDelay=250) {	//TODO: stop short
	//move lift out of way of intake
	if (pulseIntake) setPower(coneIntake, 60);
	moveLiftToSafePos();

	if (pulseIntake) setPower(coneIntake, INTAKE_STILL_SPEED);
	moveGoalIntake(in, true);

	if (in)
		wait1Msec(intakeDelay);

	if (quadRamp)
		driveStraight(dist, true, 50, 120, -20, 250, 20, false);
	else
		driveStraight(dist, true);

	while (goalDirection != 0) EndTimeSlice();

	if (stackCone) stackNewCone();

	while (driveData.isDriving || (stacking && stackCone)) EndTimeSlice();
}

void sideGoal(bool retract=true, bool twentyPt=true, bool middle=true) {	//gets mobile goal on parking tile in front of robot and scores with preload in 10pt or 20pt zone (depending on argument)
	//pick up side goal
	driveAndGoal(15, false, false, true, true);
	driveStraight(30);

	//position robot so it is ready to outtake goal into 20pt zone
	driveAndGoal(-42, true, true);

	moveLiftToSafePos();

	turn(-45);
	driveStraight(middle ? -23 : -10);
	turn(-90);

	scoreGoal(retract, twentyPt);
}

void middleGoal(bool left, bool twentyPt, bool center=true) {
	int direction = (left ? 1 : -1);

	if (left) setPower(coneIntake, 60);
	moveLiftToSafePos();
	if (left) setPower(coneIntake, INTAKE_STILL_SPEED);
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

task skillz() {
	turnDefaults.reversed = false;

	middleGoal(true, true);	//near left middle goal

	turn(-95);	//TODO: turnToLine() (& similar below)
	driveStraight(GOAL_TO_MID_DIST);
	turn(-95);

	middleGoal(false, false, false);	//near right middle goal

	/*turn(-90);
	driveStraight(15);
	turn(-90);*/
	turn(-150);
	driveStraight(5);
	turn(-30);

	//far right middle goal
	driveStraight(60);	//push aside cones
	driveStraight(-GOAL_TO_MID_DIST);

	moveGoalIntake(false);
	driveStraight(20);

	driveAndGoal(30, true);

	turn(-90);
	driveStraight(GOAL_TO_MID_DIST);
	turn(90);

	scoreGoal();

	turn(-90);
	driveStraight(GOAL_TO_MID_DIST);
	turn(-90);

	middleGoal(false, false, false);

	turn(-90);
	driveStraight(20);
	turn(-45);

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

task autonomous() {
	resetEncoders();
	stopLiftTargeting();
	startTask(autoStacking);
	numCones = 0;

	if (SKILLZ_MODE) {
		startTask(skillz);
	}
	else {
		turnDefaults.reversed = SensorValue[sidePot]<1830;
		startTask(sideGoalTask);
	}

	while (true) {
		executeLiftManeuvers();
		logSensorVals();
		EndTimeSlice();
	}
}
//#endregion

//#region usercontrol
void adjustConeCount() {	//change cone count based on user input
	if (newlyPressed(resetBtn))
		numCones = 0;

	if (numCones<=MAX_NUM_CONES && newlyPressed(increaseConesBtn))
		numCones++;

	if (numCones>0 && newlyPressed(decreaseConesBtn))
		numCones--;
}

void setAutopositionState(AutoPosState state) {
	stacking = false;	//TODO: ?
	posState = state;
	startTask(autoStacking);	//reset task progress

	switch (state) {
		case NO_POS:
			stopLiftTargeting();
			break;
		case CHAIN_DEF:
			setChainBarState(CH_DEF);
			break;
		case CH_STACK:
			setChainBarState(STACK);
			break;
		case FULL_DEF:
			setChainBarState(CH_DEF);
			setLiftState(L_DEF);
			break;
		case MAX:
			setLiftState(L_MAX);
			break;
	}
}

void handleAutopositioningInput() {
	if (vexRT[chainDefBtn] == 1) {
		if (vexRT[chainStackBtn]==1 && posState!=FULL_DEF)
			setAutopositionState(FULL_DEF);
		else if (posState!=CHAIN_DEF && posState!=FULL_DEF)
			setAutopositionState(CHAIN_DEF);
	}
	else if (vexRT[chainStackBtn] == 1) {
		if (posState!=CH_STACK && posState!=FULL_DEF)
			setAutopositionState(CH_STACK);
	}
	/*else if (vexRT[maxPosBtn] == 1) {
		if (posState != MAX)
			setAutopositionState(MAX);
	}*/
	else {
		posState = NO_POS;
	}
}

void handleConeIntakeInput(bool shift) {
	if (vexRT[coneIntakeBtn] == 1)
		if (shift)
			setPower(coneIntake, -127);
		else
			setPower(coneIntake, 127);
	else
		setPower(coneIntake, INTAKE_STILL_SPEED);
}

void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>liftPos[L_SAFE] || goalPower<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift) {
			if (vexRT[stackBtn] == 1) {
				stackNewCone();
			}
			else {
				handleAutopositioningInput();
				takeInput(chainBar, !chainBar.activelyMaintining); //will only set power if not maintaining a position
			  takeInput(lift, !lift.activelyMaintining);         //if there is input, activelyMaintaining will be set to false and normal control will resume
			}
		}

		handleConeIntakeInput(shift);
	}

	executeLiftManeuvers();
}

task usercontrol() {
	stopLiftTargeting();
	handleTesting();

	startTask(autoStacking);

	bool shift;

	while (true) {
		logSensorVals();
		shift = vexRT[shiftBtn]==1;

		if (shift) {
			adjustConeCount();

			if (newlyPressed(resetEncodersBtn))
				resetEncoders();

			if (!bSoundActive && vexRT[sayConeNumberBtn]==1)
				speakNum(numCones);

			if (newlyPressed(toggleFieldingBtn)) {
				fielding = !fielding;

				if (fielding)
					playSound(soundDownwardTones);
				else
					playSound(soundUpwardTones);

				setLiftControlMode(fielding);
			}
		}

		if (newlyPressed(abortManeuversBtn)) {
			stacking = false;
			startTask(autoStacking);
			stopLiftTargeting();
		}

		handleLiftInput(shift);
		handleGoalIntakeInput();

		driveRuntime(drive);
	}
}
//#endregion
