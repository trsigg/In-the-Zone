#pragma config(Sensor, in1,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in2,    chainPot,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Motor,  port1,           LDrive1,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           LDrive2,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           mobileGoal1,   tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           intake,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           chainMotors,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           liftMotors,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           mobileGoal2,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           RDrive1,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          RDrive2,       tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config
#define TESTING 0	//0 for normal behavior, 1 for PID testing
//#endregion

//#region positions
enum chainState  { CH_DEF,	INTAKE, SAFE, STACK };	//when chain bar is SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 2500,    2800,   1840, 760 };

enum liftState  { L_DEF, L_ZERO, L_MAX, PRELOAD, M_BASE_POS, S_BASE_POS };
int liftPos[] = { 400,   1150,   2000,  1100,    400,        1250 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\pd_autoMove.c"
#include "..\lib\buttonTracker.c"
//#endregion

//#region buttons
#define stackBtn					Btn5U
	//#subregion lift mode
#define manualModeBtn			Btn7R
#define autostackBtn			Btn7L
	//#endsubregion
	//#subregion
#define goalIntakeBtn			Btn7D
#define goalOuttakeBtn		Btn7U
	//#endsubregion
	//#subregion lift
#define liftUpBtn					Btn5U
#define liftDownBtn				Btn5D
	//#endsubregion
	//#subregion chain bar
#define chainInBtn				Btn6U
#define chainOutBtn				Btn6D
	//#endsubregion
	//#subregion cone intake
#define intakeBtn					Btn8U
#define outtakeBtn				Btn8D
	//#endsubregion
	//#subregion cone count adjustment
#define resetBtn					Btn8L
#define decreaseConesBtn	Btn8D
#define increaseConesBtn	Btn8U
	//#endsubregion
//#endregion

//#region constants
	//#subregion measurements
#define CONE_HEIGHT 3.0
#define LIFT_LEN 11.5
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED	10
#define LIFT_STILL_SPEED		10
	//#endsubregion
#define INTAKE_DURATION 300	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 200
#define RAD_TO_POT_FCTR 880.1
#define LIFT_OFFSET 2.5
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
static float heightOffset = sin((liftPos[M_BASE_POS] - liftPos[L_ZERO]) / RAD_TO_POT_FCTR);
float liftAngle1, liftAngle2, chainAngle;	//the target angles of lift sections during a stack maneuver

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
	setDriveMotors(drive, 4, LDrive1, LDrive2, RDrive1, RDrive2);
	/*attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 3.25);
	attachGyro(drive, hyro);*/

	//configure lift
	initializeGroup(lift, 1, liftMotors);
	configureButtonInput(lift, liftUpBtn, liftDownBtn, LIFT_STILL_SPEED);
  setTargetingPIDconsts(lift, 0.2, 0.001, 0.05, 25);
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 1, chainMotors);
	configureButtonInput(chainBar, chainOutBtn, chainInBtn);
	setTargetingPIDconsts(chainBar, 0.17, 0.0, 0.4, 25);
	addSensor(chainBar, chainPot);

	//configure mobile goal intake
	initializeGroup(goalIntake, 2, mobileGoal1, mobileGoal2);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);

	//configure cone intake
	initializeGroup(coneIntake, 1, intake);
	configureButtonInput(coneIntake, intakeBtn, outtakeBtn);
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
void waitForMovementToFinish(bool waitForLift=true, bool waitForChain=true, int timeout=75, float chainMargin=150, float liftMargin=150) {
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(chainBar, chainMargin) && waitForChain ||
				!errorLessThan(lift, liftMargin) && waitForLift)
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

float calcLiftTargetForHeight(float height) {
	return limit(RAD_TO_POT_FCTR * asin(height / 2 / LIFT_LEN + heightOffset) + liftPos[L_ZERO],
	             liftPos[L_DEF], liftPos[L_MAX]);
}

void stackNewCone() {	//TODO: account for limited range of motion, modulus
	float stackHeight = CONE_HEIGHT * numCones;

	chainAngle = chainPos[STACK];
	liftAngle2 = calcLiftTargetForHeight(stackHeight);
	liftAngle1 = calcLiftTargetForHeight(stackHeight + LIFT_OFFSET);
	stacking = true;
}

void executeLiftManeuvers() {
	maintainTargetPos(chainBar);
	maintainTargetPos(lift);
}

void stopLiftTargeting() {
	stopTargeting(chainBar);
	stopTargeting(lift);
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
		setChainBarState(SAFE);
		setTargetPosition(lift, liftAngle1);

		while (getPosition(lift) < liftAngle2) EndTimeSlice();
		setTargetPosition(chainBar, chainAngle);
		waitForMovementToFinish(false);
		setTargetPosition(lift, liftAngle2);

		waitForMovementToFinish();

		//expel cone
		setPower(coneIntake, -127);
		setTargetPosition(lift, liftAngle1);
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
			abort = false;
		}

		executeLiftManeuvers();
	}
}

void handleTesting() {
	if (TESTING == 1)
		testPIDs();
}
//#endregion

//#region autonomous
task autonomous() {

}
//#endregion

task usercontrol() {
	handleTesting();

	bool manualLift = false;	//whether lifting is manual or controlled by autostacking

	startTask(autoStacking);

	while (true) {
		if (manualLift) {
			takeInput(lift);
			takeInput(chainBar);
			takeInput(coneIntake);

			if (vexRT[autostackBtn] == 1)
				manualLift = false;
		}
		else {
			if (!stacking && vexRT[stackBtn]==1)
				stackNewCone();

			adjustConeCount();
			executeLiftManeuvers();

			if (vexRT[manualModeBtn] == 1)
				manualLift = true;
		}

		takeInput(goalIntake);
		driveRuntime(drive);
	}
}
