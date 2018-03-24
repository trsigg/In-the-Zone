#ifndef RUN_AUTON_AS_MAIN
	#include "Vex_Competition_Includes.c"
#endif

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"
#include "..\control\commonControl.c"


void initializeGyro() {
	SensorType[gyro] = sensorNone;
	wait1Msec(2000);
	SensorType[gyro] = sensorGyro;

	SensorScale[ hyro[robot] ] = 140;
	SensorBias[ hyro[robot] ] = 1858;
}

void pre_auton() {
	#ifndef RUN_AUTON_AS_MAIN
		bStopTasksBetweenModes = true;
	#endif

	initializeStructs();

	initializeAutoMovement();
	driveDefaults.debugStartCol = debugParameters[4];
	turnDefaults.debugStartCol = debugParameters[5];

	#ifdef HAS_SPEAKER
		initializeAudio();
	#endif
}
