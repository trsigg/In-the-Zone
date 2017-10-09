#pragma config(Sensor, in1,    chainPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
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
enum chainState  { CH_DEF,	INTAKE, SAFE, STACK, CH_MIN, VERT, CH_MAX };	//when chain bar is SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 1000,    800,    1800, 3000,  580,    2680, 4050 };

enum liftState  { L_DEF, L_ZERO, L_MIN, L_MAX, PRELOAD, M_BASE_POS, S_BASE_POS };
int liftPos[] = { 1230,  1575,   1220,  2360,  525,     1100,       900 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\pd_autoMove.c"
#include "..\lib\buttonTracker.c"
//#endregion

//#region buttons
	//#subregion lift mode
#define manualModeBtn			Btn7R
#define autostackBtn			Btn7L
	//#endsubregion

	//#subregion goal intake
#define goalIntakeBtn			Btn7D
#define goalOuttakeBtn		Btn7U
	//#endsubregion

		//#subsubregion cone intake
#define intakeBtn					Btn6U
#define outtakeBtn				Btn6D
		//#endsubsubregion

	//#subregion autopositioning
#define chainDefBtn				Btn8D	//takes chain bar to CH_DEF
#define chainStackBtn			Btn8L //takes chain bar to STACK
                                //when pressed together, they take lift to L_DEF and chain bar to CH_DEF
	//#endsubregion

	//#subregion autostacking control
#define stackBtn					Btn5U
		//#subsubregion cone count adjustment
#define resetBtn					Btn5D
#define increaseConesBtn	Btn8U
#define decreaseConesBtn	Btn8R
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
	//#subregion measurements
#define CONE_HEIGHT 2.2
#define LIFT_LEN 14.0
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED	15
#define LIFT_STILL_SPEED		10
#define CHAIN_STILL_SPEED		15
#define GOAL_STILL_SPEED		15
	//#endsubregion
#define APATHY_CONES 0	//number of cones for which lift does not move
#define RECKLESS_CONES 5	//number of cones for which chain bar goes directly to STACK (not SAFE first)
#define MAX_NUM_CONES 16
#define INTAKE_DURATION 500	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 200
#define RAD_TO_POT_FCTR 880.1
#define LIFT_OFFSET 1.5
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
bool manualLift = true;	//manual or automatic lift control?
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
	/*attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 3.25);
	attachGyro(drive, hyro);*/

	//configure lift
	initializeGroup(lift, 1, liftMotors);
	configureButtonInput(lift, liftUpBtn, liftDownBtn);
	configureBtnDependentStillSpeed(lift, LIFT_STILL_SPEED);
  setTargetingPIDconsts(lift, 0.4, 0.002, 1.5, 25);	//.1/.35/.27, .001/.003/.003, .05/1.7/2.5
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 2, chain1, chain2);	//TODO: setAbsolutes
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
	configureButtonInput(coneIntake, intakeBtn, outtakeBtn, INTAKE_STILL_SPEED);
}

//#region lift
void setLiftState(liftState state) {
	setTargetPosition(lift, liftPos[state]);
}
//#endregion

//#region chain bar
void setChainBarState(chainState state) {
	setTargetPosition(chainBar, chainPos[state]);
}
//#endregion

//#region autostacking
void waitForMovementToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=75, float chainMargin=200, float liftMargin=200) {
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

	if (autoStillSpeed && lift.posPID.target<=liftPos[L_DEF] && errorLessThan(lift, 25) && lift.activelyMaintining)
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
		setChainBarState(INTAKE);
		setLiftState(L_DEF);
		setPower(coneIntake, 127);
		waitForMovementToFinish(true, true, INTAKE_DURATION);
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(numCones<=RECKLESS_CONES ? STACK : SAFE);
		setTargetPosition(lift, liftAngle1);

		while (getPosition(lift)<liftAngle2 && !errorLessThan(lift, 200)) EndTimeSlice();
		if (numCones > RECKLESS_CONES) setChainBarState(STACK);
		waitForMovementToFinish(false);
		setTargetPosition(lift, liftAngle2, false);	//change target without resetting integral

		waitForMovementToFinish(true, true, 250);
		//wait1Msec(500);

		//expel cone
		setPower(coneIntake, -127);
		setTargetPosition(lift, liftAngle1, false);
		waitForMovementToFinish(true, false, OUTTAKE_DURATION);
		setChainBarState(CH_DEF);
		setPower(coneIntake, 0);

		//return to ready positions
		numCones++;
		stacking = false;
		setLiftState(L_DEF);
	}
}

void adjustConeCount() {	//change cone count based on user input
	if (newlyPressed(resetBtn))
			numCones = 0;

	if (newlyPressed(increaseConesBtn))
			numCones++;

	if (newlyPressed(decreaseConesBtn))
			if (numCones > 0)
				numCones--;
}
//#endregion

//#region testing
int targets[] = { 0, 0 };	//chain bar, lift
bool abort = false;
bool end = false;

void testPIDs() {
	int prevTargets[] = { 0, 0 };

	while (!end) {
		if (targets[0] != prevTargets[0]) {
			setTargetPosition(chainBar, targets[0]);
			prevTargets[0] = targets[0];
		}

		if (targets[1] != prevTargets[1]) {
			setTargetPosition(lift, targets[1]);
			prevTargets[1] = targets[1];
		}

		if (abort) {
			stopLiftTargeting();
			setPower(lift, 0);
			setPower(chainBar, 0);
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
task autonomous() {

}
//#endregion

//#region usercontrol
void setAutopositionState(AutoPosState state) {
	stacking = false;
	posState = state;
	if (!manualLift) startTask(autoStacking);	//reset task progress

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

void handleManualInput() {
	handleAutopositioningInput();

	takeInput(chainBar, !chainBar.activelyMaintining);	//will only set power if not maintaining a position
	takeInput(lift, !lift.activelyMaintining);					//if there is input, activelyMaintaining will be set to false and normal control will resume

	takeInput(coneIntake);

	if (vexRT[autostackBtn] == 1) {	//switch to autostacking mode
		manualLift = false;
		startTask(autoStacking);
	}
}

void handleAutostackInput() {
	if (!stacking) {
		if (vexRT[stackBtn] == 1) {
			stackNewCone();
		}
		else {
			takeInput(coneIntake);
			handleAutopositioningInput();
		}
	}

	adjustConeCount();

	if (vexRT[manualModeBtn] == 1) {	//switch to manual mode
		stacking = false;
		stopLiftTargeting();
		stopTask(autoStacking);
		manualLift = true;
	}
}

task usercontrol() {
	handleTesting();

	if (!manualLift) startTask(autoStacking);

	while (true) {
		if (manualLift)
			handleManualInput();
		else
			handleAutostackInput();

		executeLiftManeuvers();

		takeInput(goalIntake);
		driveRuntime(drive);
	}
}
//#endregion
