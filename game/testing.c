#include "lift.c"

void logSensorVals() {
	if (debugParameters[1] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[1], getPosition(lift));

	if (debugParameters[6] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[6], SensorValue[coneSonar[robot]]);
}


//#region PID testing
#define NUM_INPUTS 9
long testingTimer;
int debugOut;
int targets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };	/*0-lift, 1-driveStraight, 2-turn, 3-lift PID mode (up=1),
                                                            4-goalIntake maneuver target, 5-stack nth cone, 6-move fb (in=1),
																														7-gyro scale, 8-drive maxAcc100ms*/
bool waite = true; //TODO: fix for lift maneuvers
bool abort = false;
bool end = false;

void handlePIDinput(int index) {
	int input = targets[index];

	if (waite) testingTimer = resetTimer();
	bool timedManeuver= false;

	switch (index) {
		case 0:
			timedManeuver = true;
			setTargetPosition(lift, input);
			//if (waite) waitForMovementToFinish(lift);
			break;
		case 1:
			timedManeuver = true;
			driveStraight(input, !waite);
			break;
		case 2:
			timedManeuver = true;
			turn(input, !waite);
			break;
		case 3:
			if (MULTIPLE_PIDs)
				setLiftPIDmode(input == 1);
			break;
		case 4:
			timedManeuver = true;
			createManeuver(goalIntake, input/*, !waite*/);
			//moveGoalIntake(input == 1);
			break;
		case 5:
			if (input > 0) {
				timedManeuver = true;
				numCones = input - 1;
				stackNewCone(/*waite*/false, false);
			}
			break;
		case 6:
			moveFourBar(input==1/*, !waite*/);
			break;
		case 7:
			SensorScale[ hyro[robot] ] = input;
			break;
		case 8:
			configureRamping(drive, input);
			break;
	}

	if (waite && timedManeuver) {
		debugOut = time(testingTimer);
		playSound(soundLowBuzz);
	}
}

void testPIDs() {
	int prevTargets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };	//TODO: fill automatically

	while (!end) {
		debugOut = SensorScale[ hyro[robot] ];

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
	while (!end) {
		datalogAddValueWithTimeStamp(0, SensorValue[in7]);
		datalogAddValueWithTimeStamp(1, SensorValue[in8]);
	}
}
//#endregion

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
	else if (TESTING == 3)
		miscTest();
}
