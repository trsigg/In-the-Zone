#include "lift.c"

void logSensorVals() {
	if (debugParameters[1] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[1], getPosition(lift));
}


//#region PID testing
#define NUM_INPUTS 8
int debugOut;
int targets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0 };	/*0-lift, 1-driveStraight, 2-turn, 3-lift PID mode (up=1),
                                                        4-goalIntake maneuver target, 5-stack nth cone, 6-move fb (in=1), 7-gyro scale*/
bool abort = false;
bool end = false;

void handlePIDinput(int index) {
	int input = targets[index];

	switch (index) {
		case 0:
			setTargetPosition(lift, input);
			break;
		case 1:
			driveStraight(input);
			playSound(soundLowBuzz);
			break;
		case 2:
			turn(input);
			playSound(soundLowBuzz);
		case 3:
			if (MULTIPLE_PIDs)
				setLiftPIDmode(input == 1);
			break;
		case 4:
			createManeuver(goalIntake, input);
			//moveGoalIntake(input == 1);
			break;
		case 5:
			if (input > 0) {
				numCones = input - 1;
				stackNewCone();
			}
			break;
		case 6:
			moveFourBar(input == 1);
			break;
		case 7:
			SensorScale[HYRO] = input;
	}
}

void testPIDs() {
	int prevTargets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	//arrayCopy(targets, prevTargets, NUM_INPUTS);

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
