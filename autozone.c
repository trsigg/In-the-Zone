#pragma config(Sensor, in1,    hyro,           sensorGyro)
#pragma config(Sensor, in2,    liftPot,        sensorPotentiometer)
#pragma config(Sensor, in3,    chainPot,       sensorPotentiometer)
#pragma config(Sensor, in4,    goalPot,        sensorPotentiometer)
#pragma config(Sensor, dgtl1,  leftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightEnc,       sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  coneEnc,        sensorQuadEncoder)
#pragma config(Motor,  port1,           LDrive1,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           LDrive2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           RDrive1,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           RDrive2,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           chainMotor,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           rollers,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           mobileGoal1,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           mobileGoal2,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           lift1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          lift2,         tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//#region config
#define DR4B false	//true if using double reverse as primary lift
//#endregion

//#region positions
enum chainState  { CH_DEF,	INTAKE, SAFE, STACK };	//when chain bar is SAFE, lift can move up and down without colliding with cone stack
int chainPos[] = { -10,     -15,    90,   180 };

enum liftState  { L_DEF, PRELOAD, M_BASE_POS, S_BASE_POS };
int liftPos[]	= { -30,   20,      -30,        10 };
//#endregion

//#region setup
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "lib\pd_autoMove.c"
#include "lib\buttonTracker.c"
//#endregion

//#region buttons
	//#subregion mobile goal
#define mobileGoalInBtn   Btn6U
#define mobileGoalOutBtn	Btn6D
	//#endsubregion
#define stackBtn Btn5U
//#endregion

//#region constants
  //#subregion measurements
#define CHAIN_BAR_OFFSET -1	//offset from the top of the cone stack when chain bar starts rotating from safe to its final position (to avoid collision)
                            //measured in
  //#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 10
	//#endsubregion
#define INTAKE_DURATION 300	//amount of time rollers activate when intaking/expelling
#define potToRadFactor 0.001136
//#endregion

//#region globals
int numCones = 0; //current number of stacked cones
bool stacking = false;	//whether the robot is currently in the process of stacking
double liftAngle, chainAngle;	//the target angles of lift sections during a stack maneuver

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
	attachEncoder(drive, leftEnc, LEFT);
	attachEncoder(drive, rightEnc, RIGHT, false, 3.25);
	attachGyro(drive, hyro);

	//configure lift
	initializeGroup(lift, 2, lift1, lift2);
  setTargetingPIDconsts(lift, 0.3, 0.0005, 0.15, 25);
	addSensor(lift, liftPot);

	//configure chain bar
	initializeGroup(chainBar, 1, chainMotor);
	setTargetingPIDconsts(chainBar, 0.2, 0.0005, 0.03, 25);
	addSensor(chainBar, chainPot);

	//configure mobile goal intake
	initializeGroup(goalIntake, 2, mobileGoal1, mobileGoal2);
	addSensor(goalIntake, goalPot);

	//configure cone intake
	initializeGroup(coneIntake, 1, rollers);
	addSensor(coneIntake, coneEnc);
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
void waitForMovementToFinish(bool waitForChain=true, bool waitForLift=true, int timeout=75, double chainMargin=7, double liftMargin=10) {
	long movementTimer = resetTimer();

	while (time(movementTimer) < timeout) {
		if (!errorLessThan(chainBar, chainMargin) && waitForChain ||
				!errorLessThan(lift, liftMargin) && waitForLift)
			movementTimer = resetTimer();
		EndTimeSlice();
	}
}

void stackNewCone() {	//TODO: account for limited range of motion
	chainAngle = chainPos[STACK];
	liftAngle = asin(potToRadFactor * getPosition(lift) + liftPos[M_BASE_POS]);
}

task liftManeuvers() {
	while (true) {
		maintainTargetPos(chainBar);
		maintainTargetPos(lift);
		EndTimeSlice();
	}
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
		setChainBarState(CH_DEF);
		wait1Msec(INTAKE_DURATION);
		setPower(coneIntake, 0);

		//return to ready positions
		setLiftState(L_DEF);
	}
}
//#endregion

//#region autonomous
task autonomous() {

}
//#endregion

task usercontrol() {
	startTask(liftManeuvers);
	startTask(autoStacking);

	while (true) {
		if (newlyPressed(stackBtn))
			stackNewCone();

		driveRuntime(drive);
	}
}
