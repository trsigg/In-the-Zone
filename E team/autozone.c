#pragma config(Sensor, in1,    chainPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    hyro,           sensorGyro)
#pragma config(Sensor, in4,    leftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    rightLine,      sensorReflection)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           RDrive1,       tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           RDrive2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           chain1,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           intake,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           liftMotors,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           LDrive1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           chain2,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           goal1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          goal2,         tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
//#endregion

//#region positions
enum chainState  { CH_FIELD, INTAKE, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 700,      700,    1800,    2800,  580,    2680, 4050 };

enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, PRELOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
int liftPos[] = { 1380,  1400,    1600,   1380,       1530,    1575,   2670 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\pd_autoMove.c"
#include "..\lib\buttonTracker.c"
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
#define liftUpBtn					Btn5U
#define liftDownBtn				Btn5D
		//#endsubsubregion
		//#subsubregion chain bar
#define chainInBtn				Btn8U
#define chainOutBtn				Btn8R
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT_FCTR 880.1
#define LINE_THRESHOLD  300
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 2.5
#define LIFT_LEN    14.0
#define LIFT_OFFSET 2.25
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 15
#define LIFT_STILL_SPEED   10
#define CHAIN_STILL_SPEED  15
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES   0	//number of cones for which lift does not move
#define RECKLESS_CONES 2	//number of cones for which chain bar goes directly to STACK (not CH_SAFE first)
#define MAX_NUM_CONES  16
	//#endsubregion
	//#subregion timing
#define INTAKE_DURATION  400	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 350
	//#endsubregion
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
bool preload = false;	//whether robot is intaking cones from the preload or field
static float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_POT_FCTR);	//used in autostacking
float liftAngle1, liftAngle2;	//the target angles of lift sections during a stack maneuver

	//#subregion autopositioning
enum AutoPosState { NO_POS, CHAIN_DEF, CH_STACK, FULL_DEF };
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

	//configure drive
	initializeDrive(drive, true);
	setDriveMotors(drive, 4, LDrive1, LDrive1, RDrive1, RDrive2);
	attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 4.0);
	attachGyro(drive, hyro);

	//configure lift (PID handled in setLiftPIDmode)
	initializeGroup(lift, 1, liftMotors);
	configureButtonInput(lift, liftUpBtn, liftDownBtn);
	configureBtnDependentStillSpeed(lift, LIFT_STILL_SPEED);
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 2, chain1, chain2);
	setAbsolutes(chainBar, chainPos[CH_MIN], chainPos[CH_MAX]);
	configureButtonInput(chainBar, chainInBtn, chainOutBtn);
	configurePosDependentStillSpeed(chainBar, CHAIN_STILL_SPEED, chainPos[VERT]);
	setTargetingPIDconsts(chainBar, 0.23, 0.001, 0.55, 25);	//0.2/.3, 0.001, 0.15/.7
	addSensor(chainBar, chainPot);

	//configure mobile goal intake
	initializeGroup(goalIntake, 2, goal1, goal2);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);

	//configure cone intake
	initializeGroup(coneIntake, 1, intake);
}

//#region audio
void speakNum(int num) {
	string fileName;
	if (num >= 10)
		strcat(fileName, "1");
	char onesDigit[] = { '0' + (num % 10) };
	strcat(fileName, onesDigit);
	strcat(fileName, ".wav");

	playSoundFile(fileName);
}
//#endregion

//#region lift
void setLiftPIDmode(bool up) {	//up is true for upward movement consts, false for downward movement ones.
	if (up)
		setTargetingPIDconsts(lift, 0.4, 0.002, 1.5, 25);	//.1/.35/.27, .001/.003/.003, .05/1.7/2.5
	else
		setTargetingPIDconsts(lift, 0.4, 0.002, 1.5, 25);
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
void setChainBarState(chainState state) {
	if (state == CH_DEF)
		setChainBarState(preload ? CH_SAFE : CH_FIELD);
	else
		setTargetPosition(chainBar, chainPos[state]);
}
//#endregion

//#region autostacking
void waitForMovementToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=100, float chainMargin=200, float liftMargin=200) {
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

void executeLiftManeuvers(bool autoStillSpeed=true) {
	maintainTargetPos(chainBar);

	if (autoStillSpeed && lift.posPID.target<=liftPos[L_FIELD] && errorLessThan(lift, 25) && lift.activelyMaintining)
		setPower(lift, -LIFT_STILL_SPEED);
	else
		maintainTargetPos(lift);
}

void stopLiftTargeting() {
	stopTargeting(chainBar);
	setPower(chainBar, 0);

	stopTargeting(lift);
	setPower(lift, 0);
}

task autoStacking() {
	while (true) {
		while (!stacking) EndTimeSlice();

		//intake cone
		/*setLiftState(L_DEF);
		setChainBarState(INTAKE);
		setPower(coneIntake, 127);
		waitForMovementToFinish(true, true, INTAKE_DURATION);*/
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(numCones<=RECKLESS_CONES ? STACK : CH_SAFE);
		setLiftTargetAndPID(liftAngle1);

		while (!errorLessThan(lift, 200)) EndTimeSlice();
		if (numCones > RECKLESS_CONES) setChainBarState(STACK);
		waitForMovementToFinish(false);
		setLiftTargetAndPID(liftAngle2, false);	//change target without resetting integral

		waitForMovementToFinish(true, true, 250);

		if (numCones <= 14) {
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
		}
	}
}
//#endregion

//#region testing
int targets[] = { 0, 0, 0, 0 };	//chain bar, lift, driveStraight, turn
bool abort = false;
bool end = false;

void testPIDs() {
	int prevTargets[] = { 0, 0, 0, 0 };

	while (!end) {
		if (targets[0] != prevTargets[0]) {
			setTargetPosition(chainBar, targets[0]);
			prevTargets[0] = targets[0];
		}

		if (targets[1] != prevTargets[1]) {
			setLiftTargetAndPID(targets[1]);
			prevTargets[1] = targets[1];
		}

		if (targets[2] != prevTargets[2]) {
			//driveStraight(targets[2]);
			prevTargets[2] = targets[2];
		}

		if (targets[3] != prevTargets[3]) {
			turn(targets[3]);
			prevTargets[3] = targets[3];
		}

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
void alignToLine(int power=80) {
	setDrivePower(drive, power, power);
	bool leftRunning=true, rightRunning=true;

	while (leftRunning || rightRunning) {
		if (leftRunning && SensorValue[leftLine]<LINE_THRESHOLD) {
			setLeftPower(drive, 0);
			leftRunning = false;
		}

		if (rightRunning && SensorValue[rightLine]<LINE_THRESHOLD) {
			setRightPower(drive, 0);
			rightRunning = false;
		}
	}
}

task autonomous() {

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

void handleGoalIntakeInput(bool shift) {
	int goalPower = takeInput(goalIntake, false);

	if (!shift && (getPosition(lift)>liftPos[L_SAFE] || goalPower<=GOAL_STILL_SPEED))
		setPower(goalIntake, goalPower);
}

void handleLiftInput(bool shift) {
	if (!stacking) {
		if (!shift && vexRT[stackBtn]==1) {
			stackNewCone();
		}
		else {
			handleConeIntakeInput(shift);

			takeInput(lift, !lift.activelyMaintining); //will only set power if not maintaining a position
								                                 //if there is input, activelyMaintaining will be set to false and normal control will resume
			if (!shift) {
				takeInput(chainBar, !chainBar.activelyMaintining);
				handleAutopositioningInput();
			}
		}
	}

	executeLiftManeuvers();
}

task usercontrol() {
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

		handleLiftInput(shift);
		handleGoalIntakeInput(shift);

		driveRuntime(drive);
	}
}
//#endregion
