#pragma config(Sensor, in1,    hyro,           sensorGyro)
#pragma config(Sensor, in3,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in4,    leftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    rightLine,      sensorLineFollower)
#pragma config(Sensor, in6,    backLine,       sensorLineFollower)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  chainEnc,       sensorQuadEncoder)
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
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
int testingParameters[] = { -1, -1 };	//testPIDs: { liftDebugStartCol, chainDebugStartCol }
//#endregion

//#region positions
enum chainState  { CH_FIELD, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 145,      97,      32,    0,      56,   247 };

enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, PRELOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
int liftPos[] = { 1270,  1270,    1610,   1270,       1515,    1600,   2400 };
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
#define togglePreloadBtn  Btn7D
		//#subsubregion cone count adjustment
#define resetBtn					Btn8R	//with shift
#define increaseConesBtn	Btn8U	//with shift
#define decreaseConesBtn	Btn8D	//with shift
		//#endsubsubregion
	//#endsubregion

//#subregion manual control
		//#subsubregion lift
#define liftUpBtn					Btn8U
#define liftDownBtn				Btn8R
		//#endsubsubregion
		//#subsubregion chain bar
#define chainInBtn				Btn5U
#define chainOutBtn				Btn5D
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT_FCTR 880.1
#define LINE_THRESHOLD  300
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 2.0
#define LIFT_LEN    14.0
#define LIFT_OFFSET 2.0
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 15
#define LIFT_STILL_SPEED   10
#define L_AUTO_SS_MARGIN   50
#define CHAIN_STILL_SPEED  15
#define CH_AUTO_SS_MARGIN	 0
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES    0 //number of cones for which lift does not move
#define RECKLESS_CONES  2 //number of cones for which chain bar goes directly to STACK (not CH_SAFE first)
#define NO_OFFSET_CONES 1 //number of cones for which the lift goes straight to liftAngle2
#define MAX_NUM_CONES   15
	//#endsubregion
	//#subregion timing
#define INTAKE_DURATION  400	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 450
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 2250
	//#endsubregion
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
bool preload = false;	//whether robot is intaking cones from the preload or field
static float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_POT_FCTR);	//used in autostacking
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

void pre_auton() {
	bStopTasksBetweenModes = true;

	initializeAutoMovement();
	driveDefaults.kP_c = 0;	//temporary (due to encoder issues)
	driveDefaults.kI_c = 0;
	driveDefaults.kD_c = 0;

	//configure drive
	initializeDrive(drive, true);
	setDriveMotors(drive, 4, LDrive1, LDrive1, RDrive1, RDrive2);
	attachEncoder(drive, leftEnc, LEFT, true);
	attachEncoder(drive, rightEnc, RIGHT, true, 4.0, 2.0);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 2, lift1, lift2);
	configureButtonInput(lift, liftUpBtn, liftDownBtn);
	configureBtnDependentStillSpeed(lift, LIFT_STILL_SPEED);
	initializeTargetingPID(lift, 0, 0, 0, 25);	//gain setup in setLiftPIDmode
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 2, chain1, chain2);
	//setAbsolutes(chainBar, chainPos[CH_MIN], chainPos[CH_MAX]);
	configureButtonInput(chainBar, chainOutBtn, chainInBtn);
	configurePosDependentStillSpeed(chainBar, CHAIN_STILL_SPEED, chainPos[VERT]);
	initializeTargetingPID(chainBar, 0, 0, 0, 25);	//gain setup in setChainBarPIDmode
	addSensor(chainBar, chainEnc, true);
	configureEncoderCorrection(chainBar, chainPos[CH_MAX]);

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
		setTargetingPIDconsts(lift, 0.5, 0.001, 1.5);	//0.37, 0.002, 1.6
	else
		setTargetingPIDconsts(lift, 0.5, 0.001, 1.5);
}

void setLiftTargetAndPID(int target, bool resetIntegral=true) {	//sets lift target and adjusts PID consts
	if (getPosition(lift) < target)
		setLiftPIDmode(true);
	else
		setLiftPIDmode(false);

	setTargetPosition(lift, target, resetIntegral);
}

void setLiftState(liftState state) {
	if (state == L_DEF)
		setLiftState(preload ? PRELOAD : L_FIELD);
	else
		setLiftTargetAndPID(liftPos[state]);
}
//#endregion

//#region chain bar
void setChainBarPIDmode(bool low) {	//	low should be true for targets below VERT
	if (low)
		setTargetingPIDconsts(chainBar, 3.1, 0.015, 11);
	else
		setTargetingPIDconsts(chainBar, 3.1, 0.015, 11);
}

void setChainBarTargetAndPID(int target, bool resetIntegral=true) {
	if (target < chainPos[VERT])
		setChainBarPIDmode(true);
	else
		setChainBarPIDmode(false);

	setTargetPosition(chainBar, target, resetIntegral);
}

void setChainBarState(chainState state) {
	if (state == CH_DEF)
		setChainBarState(preload ? CH_SAFE : CH_FIELD);
	else
		setChainBarTargetAndPID(chainPos[state]);
}
//#endregion

//#region autostacking
void waitForMovementToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=100, float chainMargin=15, float liftMargin=200) {
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
	return limit(RAD_TO_POT_FCTR * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_MIN], liftPos[L_MAX]);
}

void stackNewCone() {	//TODO: account for limited range of motion, modulus
	float stackHeight = CONE_HEIGHT * adjustedNumCones();

	liftAngle1 = calcLiftTargetForHeight(stackHeight + LIFT_OFFSET);
	liftAngle2 = calcLiftTargetForHeight(stackHeight);
	stacking = true;
}

void executeLiftManeuvers(bool autoStillSpeed=true, int liftDebugStartCol=-1, int chainDebugStartCol=-1) {
	if (autoStillSpeed && errorLessThan(chainBar, CH_AUTO_SS_MARGIN) && chainBar.activelyMaintining)
		setPower(chainBar, CHAIN_STILL_SPEED * (chainBar.posPID.target<=chainPos[VERT] ? 1 : -1));
	else
		maintainTargetPos(chainBar, chainDebugStartCol);

	if (autoStillSpeed && errorLessThan(lift, L_AUTO_SS_MARGIN) && lift.activelyMaintining)
		setPower(lift, LIFT_STILL_SPEED * (lift.posPID.target<=liftPos[L_FIELD] ? -1 : 1));
	else
		maintainTargetPos(lift, liftDebugStartCol);
}

void stopLiftTargeting() {
	stopTargeting(chainBar);
	setPower(chainBar, 0);

	stopTargeting(lift);
	setPower(lift, 0);
}

task autoStacking() {
	bool useOffset;

	while (true) {
		while (!stacking) EndTimeSlice();

		useOffset = (numCones >= NO_OFFSET_CONES);

		//intake cone
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(numCones<=RECKLESS_CONES ? STACK : CH_SAFE);
		setLiftTargetAndPID(useOffset ? liftAngle1 : liftAngle2);

		while (!errorLessThan(lift, 200)) EndTimeSlice();
		if (numCones > RECKLESS_CONES) setChainBarState(STACK);
		waitForMovementToFinish(false);
		if (useOffset) setLiftTargetAndPID(liftAngle2/*, false*/);	//change target without resetting integral

		waitForMovementToFinish(true, true, 250);

		if (numCones < MAX_NUM_CONES-1) {
			//expel cone
			setPower(coneIntake, -127);
			setLiftTargetAndPID(liftAngle1, false);
			waitForMovementToFinish(true, false, OUTTAKE_DURATION);
			setChainBarState(CH_DEF);
			setPower(coneIntake, 0);

			//return to ready positions
			numCones++;
			stacking = false;
			while (getPosition(chainBar) > chainPos[CH_SAFE]) EndTimeSlice();
			setLiftState(L_DEF);
		}
		else {
			numCones++;
			stacking = false;
			setPower(lift, -LIFT_STILL_SPEED);	//allows lift to fall down on stack
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

		if (testingParameters[0] >= 0)
			datalogAddValueWithTimeStamp(testingParameters[0]+7, getPosition(lift));
		if (testingParameters[1] >= 0)
			datalogAddValueWithTimeStamp(testingParameters[1]+7, getPosition(chainBar));

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

		executeLiftManeuvers(TESTING==1, testingParameters[0], testingParameters[1]);
	}
}

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
}
//#endregion

//#region autonomous
void alignToLine(int power=60, int brakePower=20, int brakeDuration=250) {	//brakepower is absolute value (sign automatically determined)
	long leftTimer, rightTimer;
	setDrivePower(drive, power, power);

	int leftProgress=0, rightProgress=0;	//0 - moving
	                                      //1 - braking
	                                      //2 - finished

	while (leftProgress<2 || rightProgress<2) {
		if (leftProgress == 0) {
			if (SensorValue[leftLine] < LINE_THRESHOLD) {
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
			if (SensorValue[rightLine] < LINE_THRESHOLD) {
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
		while (SensorValue[leftLine]<LINE_THRESHOLD && SensorValue[rightLine]<LINE_THRESHOLD) EndTimeSlice();
	else
		while (SensorValue[backLine] < LINE_THRESHOLD) EndTimeSlice();

	setDrivePower(drive, -brakePower*sgn(power), brakePower*sgn(power));

	wait1Msec(brakeDuration);

	setDrivePower(drive, 0, 0);
}

void turnQuicklyToLine(bool clockwise, bool parallelToLine) {
	int direction = clockwise ? 1 : -1;
	turn(30 * direction);
	turnToLine(parallelToLine, 40*direction);
}

void stackAndWait() {
	stackNewCone();
	while (stacking) EndTimeSlice();
}

void driveForDuration(int duration, int left=127, int right=127) {
	setDrivePower(drive, left, right);
	wait1Msec(duration);
	setDrivePower(drive, 0, 0);
}

void moveGoalIntake(bool in) {
	int direction = in ? -1 : 1;
	setPower(goalIntake, 127 * direction);
	wait1Msec(in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION);
	setPower(goalIntake, GOAL_STILL_SPEED * direction);
}

task sideGoal() {
	startTask(autoStacking);

	//move lift out of way of intake
	setPower(coneIntake, INTAKE_STILL_SPEED);
	setChainBarState(STACK);
	setLiftTargetAndPID(liftPos[L_SAFE] + 100);

	while (getPosition(lift) < liftPos[L_SAFE]) EndTimeSlice();

	moveGoalIntake(false);	//extend intake

	driveStraight(30);	//driveForDuration(1500);	//drive to mobile goal

	moveGoalIntake(true);	//retract intake
	stackNewCone();	//preload

	//position robot so it is ready to outtake goal into 20pt zone
	driveStraight(-27, true);
	while (driveData.isDriving || stacking) EndTimeSlice();

	setLiftTargetAndPID(liftPos[L_SAFE] + 100);	//lift up so mobile goal can outtake

	turn(-45);
	driveStraight(-15);
	turn(-90);
	driveForDuration(3000);

	moveGoalIntake(false);	//extend goal intake

	driveForDuration(250);	//push goal to back of zone
	driveForDuration(500, -127, -127);	//remove goal from intake

	moveGoalIntake(true);	//retract goal intake
}

task autonomous() {
	resetEncoder(chainBar);

	startTask(sideGoal);

	while (true) executeLiftManeuvers();
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
	resetEncoder(chainBar);
	handleTesting();

	startTask(autoStacking);

	bool shift;

	while (true) {
		shift = vexRT[shiftBtn]==1;

		if (shift) {
			adjustConeCount();

			if (!bSoundActive && vexRT[sayConeNumberBtn]==1)
				speakNum(numCones);

			if (newlyPressed(togglePreloadBtn)) {
				preload = !preload;

				if (preload)
					playSound(soundUpwardTones);
				else
					playSound(soundDownwardTones);
			}
		}

		if (newlyPressed(abortManeuversBtn)) {
			stacking = false;
			startTask(autoStacking);
			stopLiftTargeting();
		}

		correctEncVal(chainBar);

		handleLiftInput(shift);
		handleGoalIntakeInput();

		driveRuntime(drive);
	}
}
//#endregion
