//#define E_TEAM_PASSIVE
#define E_TEAM_ROLLER
//#define RUN_AUTON_AS_MAIN


//#region options
//#define HAS_SPEAKER
#define MULTIPLE_PIDs    false //if lift uses different PID consts for movement in different locations or directions
#define HOLD_LAST_CONE   true	//if lift stays up after stacking last cone
#define USE_ENC_CORR     true
#define DOUBLE_DRIVER    false
#define SONAR_STACKING   true
#define AUTOSTACK_CONFIG false	//using autostacking-focused button config

	//#subregion auton/skillz options
#define SKILLZ_MODE      false	//skills
#define SKILLZ_VARIANT   true
#define PARK_IN_SKILLS   false
#define CROSS_FIELD_SKLZ false
#define SKILLZ_5PT       false
#define TURN_CHEAT       true	//general
#define ABORT_AFTER_15   false
#define ANTI_MARK        1
#define ABORT_IF_NO_GOAL false
#define RETRY_GOAL_FAILS true
#define STACK_SIDE_CONES true
#define DEFENSIVE_DELAY  2000
	//#endsubregion

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID), 3 for misc testing
int debugParameters[] = { -1, 1, -1, -1, -1, -1, 0 };	//{ liftDebugStartCol, liftSensorCol, fbDebugStartCol, fbSensorCol, driveRampCol, turnRampCol, coneSonarCol }
	//#endsubregion
//#endregion

#include "..\lib\pd_autoMove.c"

//#region E Team
#ifdef E_TEAM_PASSIVE
	#include "E_PassivePragmas.c"

	#define PASSIVE

	//#subregion positions
	/*enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 565,   565,     850,    565,        1180,   1200,   2050 };	//wrong wrong wrong (check prev commits)

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 0,        0,       0,     0 };

	enum goalState  { OUT,  MID,  IN };
	int goalPos[] = { 2400, 1990, 15 };
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
	#define L_SENS_REVERSED  false	//lift
	#define FB_SENS_REVERSED false	//four bar
	#define L_ENC_REVERSED   false	//drive
	#define R_ENC_REVERSED   true

	#define HYRO          in1
	#define SIDE_POT      in2
	#define MODE_POT      in3
	#define LIFT_SENSOR   in4
	#define GOAL_SENSOR   in5
	#define GOAL_FOLLOWER in6
	#define LEFT_ENC      dgtl1
	#define RIGHT_ENC     dgtl3
	#define FRONT_SONAR   dgtl6
	#define CONE_SONAR    -1

	#define LEFT_LINE   in1	//not currently attached
	#define BACK_LINE   in1
	#define RIGHT_LINE  in1
	#define FB_SENSOR   -1	//-1 if not attached
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 14.75	//botton section-14"; top section-15.5"
	#define CONE_HEIGHT 3.5
	#define L_OFFSET    3.5
	#define GOAL_TO_MID_DIST  17	//distance from field diagonal to mid goal
	#define LINE_TO_GOAL_DIST 26	//distance from line to mid goal - TODO: wtf?
	#define BAR_TO_LINE_DIST  9
	//#endsubregion


		//#region consts
		#define SIDE_SWITCH_POS 1845	//middle of sidePos
		//#endregion

	//#subregion cone counts
	#define APATHY_CONES  0 //number of cones for which lift does not move
	#define MAX_NUM_CONES 16
	//#endsubregion

	//#subregion specific buttons
		//#subsubregion fb
	#define fbInBtn   Btn6U
	#define fbOutBtn  Btn6D
		//#endsubsubregion

		//#subsubregion autostacking control
	#define stackBtn  Btn8U
		//#endsubsubregion

		//#subsubregion autopositioning
	#define defPosBtn Btn8D	//takes lift to default position
	#define maxPosBtn Btn8L //takes lift to maximum position
		//#endsubsubregion
	//#endsubregion*/
#endif

#ifdef E_TEAM_ROLLER
	#include "E_RollerPragmas.c"

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1310,  1310,    1670,   1330,       2500,   1900,   2910 };

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 0,        0,       0,     0 };

	enum goalState  { OUT,  MID,  IN };
	int goalPos[] = { 3200, 2855, 1050 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 3
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port1, port5, port10 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_FB_MOTORS 1
	tMotor fourBarMotors[NUM_FB_MOTORS] = { port4 };

	#define NUM_LEFT_MOTORS 3
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port2, port9, port3 };

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port6, port8 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port3, port8 };

	#define NUM_ROLLER_MOTORS 1
	tMotor rollerMotors[NUM_ROLLER_MOTORS] = { port7 };
	//#endsubregion

	//#subregion sensors
	#define L_SENS_REVERSED  false	//lift
	#define FB_SENS_REVERSED false	//four bar
	#define L_ENC_REVERSED   true	//drive
	#define R_ENC_REVERSED   true

	#define HYRO          in7
	#define SIDE_POT      in4
	#define MODE_POT      in5
	#define LIFT_SENSOR   in1
	#define GOAL_SENSOR   in6
	#define GOAL_FOLLOWER in3
	#define ROLLER_ENC    -1
	#define LEFT_ENC      dgtl1
	#define RIGHT_ENC     dgtl3
	#define FRONT_SONAR   -1
	#define CONE_SONAR    dgtl5

	#define LEFT_LINE     -1	//not currently attached
	#define BACK_LINE     -1
	#define RIGHT_LINE    -1
	#define FB_SENSOR     -1
	//#endsubregion

	//#region consts
	#define SIDE_SWITCH_POS  1960	//middle of sidePos
	#define OUTTAKE_DURATION 300
	#define INTAKE_DURATION  600
	//#endregion

	//#subregion measurements
	#define LIFT_LEN 16
	#define CONE_HEIGHT 3.5
	#define L_OFFSET    4
	#define GOAL_TO_MID_DIST  18
	#define LINE_TO_GOAL_DIST 22
	#define BAR_TO_LINE_DIST  9
	//#endsubregion

	//#subregion cone counts
	#define APATHY_CONES  0 //number of cones for which lift does not move
	#define MAX_NUM_CONES 16
	//#endsubregion

	//#subregion specific buttons
		//#subsubregion rollers
	#define intakeBtn  Btn6U
	#define outtakeBtn Btn6D
		//#endsubsubregion

		//#subsubregion fb
	#define fbInBtn    Btn8D
	#define fbOutBtn   Btn8U
		//#endsubsubregion

		//#subsubregion autostacking control
	#define stackBtn   Btn6D
		//#endsubsubregion

		//#subregion autopositioning
	#define defPosBtn   Btn8R
	#define toggleFbBtn Btn8L
		//#endsubregion
	//#endsubregion
#endif
//#endregion


//#region common buttons
#define abortManeuversBtn Btn7L
#define shiftBtn          Btn7R
#define sayConeNumberBtn  Btn8L	//with shift

	//#subregion goal intake
#define goalIntakeBtn     Btn7D
#define goalOuttakeBtn    Btn7U
	//#endsubregion

	//#subregion lift
#define liftUpBtn         Btn5U
#define liftDownBtn       Btn5D
	//#endsubregion

	//#subregion autostacking control
#define toggleFieldingBtn Btn7D
		//#subsubregion cone count adjustment (all with shift)
#define resetBtn          Btn8R
#define increaseConesBtn  Btn8U
#define decreaseConesBtn  Btn8D
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
#define MAX_GOAL_RETRIES 2
	//#subregion sensor consts
#define RAD_TO_POT   880.1    //conversion factor between radians and potentiometer values
#define L_GEAR_RATIO 5	//gear ratio between lift bar angle and sensors
#define RAD_TO_ENC   (180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT =  (LIFT_SENSOR>=dgtl1 ? RAD_TO_ENC*L_GEAR_RATIO : RAD_TO_POT);
const float L_CORR_FCTR =  (LIFT_SENSOR>=dgtl1 ? RAD_TO_POT/RAD_TO_LIFT : 1);
const float FB_CORR_FCTR = (FB_SENSOR>=dgtl1 ? RAD_TO_POT/RAD_TO_ENC : 1);
#define GOAL_FOLL_THRESH  2950
#define R_LINE_THRESHOLD  2960
#define L_LINE_THRESHOLD  3060
#define B_LINE_THRESHOLD  2870
#define CONE_SONAR_THRESH 1000
	//#endsubregion
	//#subregion still speeds
#define LIFT_STILL_SPEED   15
#define L_AUTO_SS_MARGIN   50
#define FB_STILL_SPEED     20
#define FB_AUTO_SS_MARGIN  50
#define GOAL_STILL_SPEED   0
#define ROLLER_STILL_SPEED 15
	//#endsubregion
	//#subregion timing
#define FB_MOVE_DURATION      700
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1250
	//#endsubregion
//#endregion

motorGroup goalIntake;
motorGroup lift;
motorGroup fourBar;
motorGroup roller;

float generalDebug[] = { 0, 0 };

//motorGroup groupWaitList[DEF_WAIT_LIST_LEN] = { lift, fourBar, goalIntake };

void initializeStructs() {
	//arrayCopy(groupWaitList, defGroupWaitList, DEF_WAIT_LIST_LEN);
	SensorScale[HYRO] = 145;

  //drive
	initializeDrive(drive, NUM_LEFT_MOTORS, leftMotors, NUM_RIGHT_MOTORS, rightMotors, true, 40);
	attachEncoder(drive, LEFT_ENC, LEFT, L_ENC_REVERSED);
	attachEncoder(drive, RIGHT_ENC, RIGHT, R_ENC_REVERSED, 3.25);
	attachUltrasonic(drive, FRONT_SONAR);
	attachGyro(drive, HYRO);

	//lift
  initializeGroup(lift, NUM_LIFT_MOTORS, liftMotors, liftUpBtn, liftDownBtn, LIFT_STILL_SPEED);
	configureBtnDependentStillSpeed(lift);
	initializeTargetingPID(lift, 0.3*L_CORR_FCTR, 0.0001*L_CORR_FCTR, 32*L_CORR_FCTR, 75/L_CORR_FCTR);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	//configureAutoStillSpeed(lift, 30);.35, 30
	addSensor(lift, LIFT_SENSOR, L_SENS_REVERSED);
	if (LIFT_SENSOR>=dgtl1) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);
	if (SKILLZ_MODE)
		configureButtonInput(goalIntake, outtakeBtn, intakeBtn);
	else
		configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);
	addSensor(goalIntake, GOAL_SENSOR);

	//top four bar
	initializeGroup(fourBar, NUM_FB_MOTORS, fourBarMotors);
	configureButtonInput(fourBar, fbOutBtn, fbInBtn);
	configureBtnDependentStillSpeed(fourBar, FB_STILL_SPEED);

	if (FB_SENSOR >= 0) {
		initializeTargetingPID(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 13*FB_CORR_FCTR, 100/FB_CORR_FCTR);
		addSensor(fourBar, FB_SENSOR, FB_SENS_REVERSED);
		if (FB_SENSOR>=dgtl1) configureEncoderCorrection(fourBar, fbPos[FB_MAX]);
	}

	#ifndef PASSIVE
		initializeGroup(roller, NUM_ROLLER_MOTORS, rollerMotors);
		if (SKILLZ_MODE)
			configureButtonInput(roller, goalOuttakeBtn, goalIntakeBtn, ROLLER_STILL_SPEED);
		else
			configureButtonInput(roller, intakeBtn, outtakeBtn, ROLLER_STILL_SPEED);
	#endif
}
