#include "lift.c"

void logSensorVals() {
	if (debugParameters[1] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[1], getPosition(lift));
}


//#region PID testing
#define NUM_INPUTS 4
int targets[NUM_INPUTS] = { 0, 0, 0, 1 };	//lift, driveStraight, turn, lift PID mode (up=1)
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
			setLiftPIDmode(input == 1);
			break;
	}
}

void testPIDs() {
	int prevTargets[NUM_INPUTS] = { 0, 0, 0, 0 };
	//arrayCopy(targets, prevTargets, NUM_INPUTS);

	while (!end) {
		for (int i=0; i<NUM_INPUTS; i++) {
			if (prevTargets[i] != targets[i]) {
				prevTargets[i] = targets[i];
				handlePIDinput(i);
			}
		}

		logSensorVals();

		if (abort) {
			stopLiftTargeting();

			driveData.isDriving = false;
			turnData.isTurning = false;
			setDrivePower(drive, 0, 0);

			wait1Msec(50);
			abort = false;
		}

		executeLiftManeuvers(TESTING == 1);
	}
}
//#endregion

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
}
