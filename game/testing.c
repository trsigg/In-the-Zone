#include "lift.c"

void logSensorVals() {
	if (debugParameters[2] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[2], getPosition(lift));
	else if (debugParameters[3] >= 0)
		datalogAddValueWithTimeStamp(debugParameters[3], getPosition(chainBar));
}


//#region PID testing
#define NUM_INPUTS 6
int targets[NUM_INPUTS] = { 0, 0, 0, 0, 1, 1 };	//chain bar, lift, driveStraight, turn, chain PID mode(low=0), lift PID mode (up=1)
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
	int prevTargets[NUM_INPUTS] = { 0, 0, 0, 0, 0, 0 };
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
//#endregion

void handleTesting() {
	if (TESTING>0 && TESTING<=2)
		testPIDs();
}
