//#region setup
#ifndef RUN_AUTON_AS_MAIN
	#include "Vex_Competition_Includes.c"
#endif

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "..\lib\buttonTracker.c"
#include "..\game\autonomous.c"

#ifdef PASSIVE
	#include "..\game\passiveControl.c"
#else
	#include "..\game\rollerControl.c"
#endif
//#endregion

void pre_auton() {
	#ifndef RUN_AUTON_AS_MAIN
		bStopTasksBetweenModes = true;
	#endif

	initializeStructs();

	initializeAutoMovement();
	driveDefaults.debugStartCol = debugParameters[4];
	turnDefaults.debugStartCol = debugParameters[5];

	if (HAS_SPEAKER)
		initializeAudio();
}
