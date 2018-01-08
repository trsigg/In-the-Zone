#include "lift.c"

void logSensorVals() {
	if (debugParameters[1] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[1], getPosition(lift));
}


//#region PID testing
#define NUM_INPUTS 8
long testingTimer;
int debugOut;
int targets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0 };	/*0-lift, 1-driveStraight, 2-turn, 3-lift PID mode (up=1),
                                                        4-goalIntake maneuver target, 5-stack nth cone, 6-move fb (in=1), 7-gyro scale*/
bool wait = false; //TODO: fix for lift maneuvers
bool abort = false;
bool end = false;

void handlePIDinput(int index) {
	int input = targets[index];

	if (wait) testingTimer = resetTimer();
	bool timedManeuver= false;

	switch (index) {
		case 0:
			timedManeuver = true;
			setTargetPosition(lift, input);
			if (wait) waitForMovementToFinish(lift);
			break;
		case 1:
			timedManeuver = true;
			driveStraight(input, !wait);
			break;
		case 2:
			timedManeuver = true;
			turn(input, !wait);
			break;
		case 3:
			if (MULTIPLE_PIDs)
				setLiftPIDmode(input == 1);
			break;
		case 4:
			timedManeuver = true;
			createManeuver(goalIntake, input, !wait);
			//moveGoalIntake(input == 1);
			break;
		case 5:
			if (input > 0) {
				timedManeuver = true;
				numCones = input - 1;
				stackNewCone(wait);
			}
			break;
		case 6:
			moveFourBar(input==1, !wait);
			break;
		case 7:
			SensorScale[HYRO] = input;
	}

	if (wait && timedManeuver) {
		debugOut = time(testingTimer);
		playSound(soundLowBuzz);
	}
}

void testPIDs() {
	int prevTargets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	debugOut = SensorScale[HYRO];

	while (!end) {
		for (int i=0; i<NUM_INPUTS; i++) {
			if (prevTargets[i] != targets[i]) {
				prevTargets[i] = targets[i];
				handlePIDinput(i);
			}
		}

		logSensorVals();

		if (abort) {
			stacking = false;
			stopLiftTargeting();

			driveData.isDriving = false;
			turnData.isTurning = false;
			setDrivePower(drive, 0, 0);

			wait1Msec(50);
			abort = false;
		}

		executeManeuvers(TESTING == 1);
	}
}
//#endregion

//#region misc test
void miscTest() {
	/*while (!end) {
		while ()
	}*/
}
//#endregion

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
	else if (TESTING == 3)
		miscTest();
}
