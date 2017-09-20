#pragma config(Sensor, in1,    chainPot,       sensorPotentiometer)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  goalSol1,      sensorDigitalOut)
#pragma config(Sensor, dgtl2,  goalSol2,      sensorDigitalOut)
#pragma config(Motor,  port1,           LDrive1,    tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           lift1,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RDrive1,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           LDrive2,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           lift2,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           intake,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           chain1,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           RDrive2,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           chain2,     tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


//#region config
#define TESTING 0	//0 for normal behavior, 1 for PID testing
//#endregion

//#region positions
enum chainState  { CH_DEF,	INTAKE, SAFE, STACK };	//when chain bar is SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { 2000,    2400,   2000, 450 };

enum liftState  { L_DEF, L_ZERO, L_MAX, PRELOAD, M_BASE_POS, S_BASE_POS };
int liftPos[] = { 1500,  2035,   3040,  2450,    1500,       2000 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "..\lib\pd_autoMove.c"
#include "..\lib\buttonTracker.c"
//#endregion

//#region buttons
#define toggleGoalIntakeBtn	Btn7D
#define stackBtn						Btn5U
	//#subregion cone count adjustment
#define resetBtn					Btn8U
#define decreaseConesBtn	Btn8D
#define increaseConesBtn	Btn8L
	//#endsubregion
//#endregion

//#region constants
	//#subregion measurements
#define LIFT_BASE_HEIGHT 0
#define CONE_HEIGHT 2.5
#define LIFT_LEN 11.5
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 10
	//#endsubregion
#define INTAKE_DURATION 500	//amount of time rollers activate when intaking/expelling
#define RAD_TO_POT_FCTR 880.1
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
float liftAngle, chainAngle;	//the target angles of lift sections during a stack maneuver

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
	initializeGroup(lift, 2, lift1, lift2);
  setTargetingPIDconsts(lift, 0.4, 0.0, 0.8, 25);
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 2, chain1, chain2);
	setTargetingPIDconsts(chainBar, 0.17, 0.0, 0.4, 25);
	addSensor(chainBar, chainPot);

	//configure cone intake
	initializeGroup(coneIntake, 1, intake);
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

//#region mobile goal intake
void toggleGoalIntakeState() {
	int state = 1 - SensorValue[goalSol1]; //toggles based on goalSol1 value
	SensorValue[goalSol1] = state;
	SensorValue[goalSol2] = state;
}
//#endregion

//#region autostacking
void waitForMovementToFinish(bool waitForChain=true, bool waitForLift=true, int timeout=75, float chainMargin=150, float liftMargin=150) {
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(chainBar, chainMargin) && waitForChain ||
				!errorLessThan(lift, liftMargin) && waitForLift)
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

void stackNewCone() {	//TODO: account for limited range of motion, modulus
	chainAngle = chainPos[STACK];
	liftAngle = limit(RAD_TO_POT_FCTR * asin((CONE_HEIGHT * numCones + LIFT_BASE_HEIGHT) / 2 / LIFT_LEN) + liftPos[M_BASE_POS]/*) % (2 * PI)*/,
										liftPos[L_DEF], liftPos[L_MAX]);
	stacking = true;
}

task liftManeuvers() {
	while (true) {
		maintainTargetPos(chainBar);
		maintainTargetPos(lift);
		EndTimeSlice();
	}
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
		setPower(coneIntake, 127);
		wait1Msec(INTAKE_DURATION);
		setPower(coneIntake, INTAKE_STILL_SPEED);

		//move to desired location
		setChainBarState(SAFE);
		setTargetPosition(lift, liftAngle);

		waitForMovementToFinish(false); //while (liftHeight() < numCones * CONE_HEIGHT + CHAIN_BAR_OFFSET) EndTimeSlice();
		setTargetPosition(chainBar, chainAngle);

		waitForMovementToFinish();

		//expel cone
		setPower(coneIntake, -127);
		wait1Msec(INTAKE_DURATION);
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
	startTask(liftManeuvers);
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
			abort = false;
		}

		EndTimeSlice();
	}

	stopTask(liftManeuvers);
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

	startTask(liftManeuvers);
	startTask(autoStacking);

	while (true) {
		if (newlyPressed(stackBtn))
			stackNewCone();

		if (newlyPressed(toggleGoalIntakeBtn))
			toggleGoalIntakeState();

		adjustConeCount()

		driveRuntime(drive);
	}
}
