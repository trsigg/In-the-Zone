#define E_TEAM


//#region options
#define SKILLZ_MODE    false
#define MULTIPLE_PIDs  false //if lift uses different PID consts for movement in different locations or directions
#define HOLD_LAST_CONE true	//if lift stays up after stacking last cone
#define HAS_SPEAKER    true

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
int debugParameters[] = { 0, 7, -1, -1 };	//{ liftDebugStartCol, liftSensorCol, fbDebugStartCol, fbSensorCol }
	//#endsubregion
//#endregion

//#region E Team
#ifdef E_TEAM
	#include "E_TeamPragmas.c"

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 650,   900,     1190,   750,        1300,   1195,   2160 };

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 500,      750,     1500,  1500 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 2
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port2, port9 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_FB_MOTORS 2
	tMotor fourBarMotors[NUM_FB_MOTORS] = { port1, port10 };

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port5, port7 };

	#define NUM_LEFT_MOTORS 2
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port4, port6 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port3, port8 };
	//#endsubregion

	//#subregion sensors
	#define L_SENS_REVERSED  false	//lift	- TODO: automatically determine sensor type based on digital or analog port
	#define FB_SENS_REVERSED false	//four bar
	#define L_ENC_REVERSED   true	//drive
	#define R_ENC_REVERSED   true

	#define HYRO        in1
	#define SIDE_POT    in2
	#define MODE_POT    in3
	#define LIFT_SENSOR in4
	#define LEFT_ENC    dgtl1
	#define RIGHT_ENC   dgtl3

	#define LEFT_LINE   in1	//not currently attached
	#define BACK_LINE   in1
	#define RIGHT_LINE  in1
	#define FB_SENSOR   in1
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 13.5
	//#endsubregion
#endif
//#endregion


//#region buttons
#define abortManeuversBtn Btn7L
#define shiftBtn          Btn7R
#define sayConeNumberBtn  Btn8L	//with shift

	//#subregion goal intake
#define goalIntakeBtn     Btn7D
#define goalOuttakeBtn    Btn7U
	//#endsubregion

	//#subregion top four bar
#define fbInBtn         Btn6U
#define fbOutBtn        Btn6D
	//#endsubregion

	//#subregion lift
#define liftUpBtn         Btn5U	//fielding mode
#define liftDownBtn       Btn5D
	//#endsubregion

	//#subregion autopositioning
#define defPosBtn         Btn8D	//takes lift to default position
#define maxPosBtn         Btn8L //takes lift to maximum position
	//#endsubregion

	//#subregion autostacking control
#define stackBtn          Btn6U
#define toggleFieldingBtn Btn8R
		//#subsubregion cone count adjustment (all with shift)
#define resetBtn          Btn8R
#define increaseConesBtn  Btn8U
#define decreaseConesBtn  Btn8D
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT   880.1    //conversion factor between radians and potentiometer values
#define L_GEAR_RATIO 5	//gear ratio between lift bar angle and sensors
#define RAD_TO_ENC   (180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT = (LIFT_SENSOR>=dgtl1 ? RAD_TO_ENC*L_GEAR_RATIO : RAD_TO_POT);
const float L_CORR_FCTR = (LIFT_SENSOR>=dgtl1 ? RAD_TO_POT/RAD_TO_LIFT : 1);
const float FB_CORR_FCTR = (FB_SENSOR>=dgtl1 ? RAD_TO_POT/RAD_TO_ENC : 1);
#define R_LINE_THRESHOLD 2960
#define L_LINE_THRESHOLD 3060
#define B_LINE_THRESHOLD 2870
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 4.0
#define GOAL_TO_MID_DIST 17.5
	//#endsubregion
	//#subregion still speeds
#define LIFT_STILL_SPEED   15
#define L_AUTO_SS_MARGIN   50
#define FB_STILL_SPEED     10
#define FB_AUTO_SS_MARGIN  50
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES       0 //number of cones for which lift does not move
#define MAX_NUM_CONES      14
	//#endsubregion
	//#subregion timing
//#define FB_MOVE_DURATION      750
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1750
	//#endsubregion
//#endregion


#include "..\lib\pd_autoMove.c" //for drive declaration

motorGroup goalIntake;
motorGroup lift;
motorGroup fourBar;

void initializeStructs() {
  //drive
	initializeDrive(drive, NUM_LEFT_MOTORS, leftMotors, NUM_RIGHT_MOTORS, rightMotors, true);
	attachEncoder(drive, LEFT_ENC, LEFT, L_ENC_REVERSED);
	attachEncoder(drive, RIGHT_ENC, RIGHT, R_ENC_REVERSED, 4.0, 2.0);
	attachGyro(drive, HYRO);

	//lift
  initializeGroup(lift, NUM_LIFT_MOTORS, liftMotors);
	configureButtonInput(lift, liftUpBtn, liftDownBtn);
	configureBtnDependentStillSpeed(lift, LIFT_STILL_SPEED);
	initializeTargetingPID(lift, 0.25*L_CORR_FCTR, 0.01*L_CORR_FCTR, 5*L_CORR_FCTR, 10);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	addSensor(lift, LIFT_SENSOR, L_SENS_REVERSED);
	if (LIFT_SENSOR>=dgtl1) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);

	//top four bar
	initializeGroup(fourBar, NUM_FB_MOTORS, fourBarMotors);
	configureButtonInput(fourBar, fbInBtn, fbOutBtn);
	configureBtnDependentStillSpeed(fourBar, FB_STILL_SPEED);
	initializeTargetingPID(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 1.3*FB_CORR_FCTR, 10);
	addSensor(fourBar, FB_SENSOR, FB_SENS_REVERSED);
	if (FB_SENSOR>=dgtl1) configureEncoderCorrection(fourBar, fbPos[FB_MAX]);
}
