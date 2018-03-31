#ifndef RUN_AUTON_AS_MAIN
	#include "Vex_Competition_Includes.c"
#endif

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"
#include "..\control\commonControl.c"


void pre_auton() {
	#ifndef RUN_AUTON_AS_MAIN
		bStopTasksBetweenModes = true;
	#endif

	initializeAutoMovement();
	driveDefaults.debugStartCol = debugParameters[4];
	turnDefaults.debugStartCol = debugParameters[5];

	initializeStructs();

	#ifdef HAS_SPEAKER
		initializeAudio();
	#endif
}
