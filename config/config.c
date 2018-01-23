#define E_TEAM_PASSIVE
//#define E_TEAM_ROLLER
//#define RUN_AUTON_AS_MAIN


//#region options
#define SKILLZ_MODE      false
#define ANTI_MARK        1
#define MULTIPLE_PIDs    false //if lift uses different PID consts for movement in different locations or directions
#define HOLD_LAST_CONE   true	//if lift stays up after stacking last cone
#define ABORT_IF_NO_GOAL true
#define RETRY_GOAL_FAILS false
#define PARK_IN_SKILLS   true
#define HAS_SPEAKER      true
#define USE_ENC_CORR     false
#define SONAR_STACKING   true

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID), 3 for misc testing
int debugParameters[] = { -1, -1, -1, -1, 0, -1 };	//{ liftDebugStartCol, liftSensorCol, fbDebugStartCol, fbSensorCol, driveRampCol, turnRampCol }
	//#endsubregion
//#endregion

//#region E Team
#ifdef E_TEAM_PASSIVE
	#include "E_PassivePragmas.c"

	#define PASSIVE true

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1400,  1470,    1700,   1500,       1910,   1915,   2950 };	//SAFE previously 1560

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 500,      750,     1500,  1500 };

	enum goalState  { OUT, MID, IN };
	int goalPos[] = { 15,  600, 2300 };
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
	//#endsubregion
#endif

#ifdef E_TEAM_ROLLER
	#include "E_RollerPragmas.c"

	#define PASSIVE false

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1400,  1470,    1700,   1500,       1910,   1915,   2950 };	//SAFE previously 1560

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 500,      750,     1500,  1500 };

	enum goalState  { OUT, MID, IN };
	int goalPos[] = { 15,  600, 2400 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 2
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port5, port8 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_FB_MOTORS 1
	tMotor fourBarMotors[NUM_FB_MOTORS] = { port2 };

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port6, port10 };

	#define NUM_LEFT_MOTORS 3
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port1, port4, port7 };

	#define NUM_GOAL_MOTORS 1
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port9 };

	#define NUM_ROLLER_MOTORS 1
	tMotor rollerMotors[NUM_ROLLER_MOTORS] = { port3 };
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
	#define GOAL_SENSOR   -1
	#define GOAL_FOLLOWER in6
	#define ROLLER_ENC    dgtl5
	#define LEFT_ENC      dgtl1
	#define RIGHT_ENC     dgtl3
	#define FRONT_SONAR   -1
	#define CONE_SONAR    dgtl7

	#define LEFT_LINE   in1	//not currently attached
	#define BACK_LINE   in1
	#define RIGHT_LINE  in1
	#define FB_SENSOR   -1	//-1 if not attached
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 16	//botton section-14"; top section-15.5"
	//#endsubregion

	//#subregion team-specific components
	#define intakeBtn  Btn6U
	#define outtakeBtn Btn6D
	motorGroup roller;
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
#define fbInBtn           Btn6U
#define fbOutBtn          Btn6D
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
#define stackBtn          Btn8U
#define toggleFieldingBtn Btn8R
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
#define SIDE_SWITCH_POS  1780	//middle of sidePos
#define GOAL_FOLL_THRESH 3006
#define R_LINE_THRESHOLD 2960
#define L_LINE_THRESHOLD 3060
#define B_LINE_THRESHOLD 2870
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 3.0
#define L_OFFSET    3.5
#define GOAL_TO_MID_DIST 16.5
#define BAR_TO_LINE_DIST 9
	//#endsubregion
	//#subregion still speeds
#define LIFT_STILL_SPEED  15
#define L_AUTO_SS_MARGIN  50
#define FB_STILL_SPEED    20
#define FB_AUTO_SS_MARGIN 50
#define GOAL_STILL_SPEED  15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES       0 //number of cones for which lift does not move
#define MAX_NUM_CONES      14
	//#endsubregion
	//#subregion timing
#define FB_MOVE_DURATION      500
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1250
	//#endsubregion
//#endregion


#include "..\lib\pd_autoMove.c" //for drive declaration

motorGroup goalIntake;
motorGroup lift;
motorGroup fourBar;

//motorGroup groupWaitList[DEF_WAIT_LIST_LEN] = { lift, fourBar, goalIntake };

void initializeStructs() {
	//arrayCopy(groupWaitList, defGroupWaitList, DEF_WAIT_LIST_LEN);
	SensorScale[HYRO] = 145;

  //drive
	initializeDrive(drive, NUM_LEFT_MOTORS, leftMotors, NUM_RIGHT_MOTORS, rightMotors, true, 40);
	attachEncoder(drive, LEFT_ENC, LEFT, L_ENC_REVERSED);
	attachEncoder(drive, RIGHT_ENC, RIGHT, R_ENC_REVERSED, 4.0);
	attachUltrasonic(drive, FRONT_SONAR);
	attachGyro(drive, HYRO);

	//lift
  initializeGroup(lift, NUM_LIFT_MOTORS, liftMotors, liftUpBtn, liftDownBtn, LIFT_STILL_SPEED);
	configureBtnDependentStillSpeed(lift);
	initializeTargetingPID(lift, 0.7*L_CORR_FCTR, 0.0001*L_CORR_FCTR, 70*L_CORR_FCTR, 50/L_CORR_FCTR);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	configureAutoStillSpeed(lift, 25);
	addSensor(lift, LIFT_SENSOR, L_SENS_REVERSED);
	if (LIFT_SENSOR>=dgtl1) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);
	if (SKILLZ_MODE)
		configureButtonInput(goalIntake, fbOutBtn, fbInBtn);
	else
		configureButtonInput(goalIntake, goalIntakeBtn, goalOuttakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);
	addSensor(goalIntake, GOAL_SENSOR);

	//top four bar
	initializeGroup(fourBar, NUM_FB_MOTORS, fourBarMotors);
	if (SKILLZ_MODE)
		configureButtonInput(fourBar, goalIntakeBtn, goalOuttakeBtn);
	else
		configureButtonInput(fourBar, fbInBtn, fbOutBtn);
	configureBtnDependentStillSpeed(fourBar, FB_STILL_SPEED);

	if (FB_SENSOR >= 0) {
		initializeTargetingPID(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 13*FB_CORR_FCTR, 100/FB_CORR_FCTR);
		addSensor(fourBar, FB_SENSOR, FB_SENS_REVERSED);
		if (FB_SENSOR>=dgtl1) configureEncoderCorrection(fourBar, fbPos[FB_MAX]);
	}

	#ifdef E_TEAM_ROLLER
		initializeGroup(roller, NUM_ROLLER_MOTORS, rollerMotors, intakeBtn, outtakeBtn, 15);
	#endif
}
