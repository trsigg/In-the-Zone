enum robotId { E_PASSIVE, E_ROLLER, E_PNEUMATIC };
robotId robot;
#define NUM_ROBOTS 3	//number of configurable robots

//#define E_TEAM_PASSIVE
//#define E_TEAM_ROLLER
#define E_TEAM_PNEUMATIC
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
int debugParameters[] = { 0, -1, -1, -1, -1, -1, -1 };	//{ liftDebugStartCol, liftSensorCol, fbDebugStartCol, fbSensorCol, driveRampCol, turnRampCol, coneSonarCol }
	//#endsubregion
//#endregion

#include "..\lib\pd_autoMove.c"

//#region sensors
	//#subregion config
bool liftSensReversed[NUM_ROBOTS]  = { false, false, false };
bool fbSensReversed[NUM_ROBOTS]    = { false, false, false };
bool L_EncReversed[NUM_ROBOTS]     = { false, true,  true };
bool R_EncReversed[NUM_ROBOTS]     = { true,  true,  true };
	//#endsubregion

	//#subregion ports
tSensors hyro[NUM_ROBOTS] =       { in1,   in7,   -1 };
tSensors liftSensor[NUM_ROBOTS] = { in4,   in1,   in1 };
tSensors goalSensor[NUM_ROBOTS] = { in5,   in6,   in2 };
tSensors fbSensor[NUM_ROBOTS] =   { -1,    -1,    -1 };
tSensors sidePot[NUM_ROBOTS] =    { in2,   in4,   -1 };
tSensors modePot[NUM_ROBOTS] =    { in3,   in5,   -1 };
tSensors leftEnc[NUM_ROBOTS] =    { dgtl1, dgtl1, dgtl3 };
tSensors rightEnc[NUM_ROBOTS] =   { dgtl3, dgtl3, dgtl1 };
tSensors coneSonar[NUM_ROBOTS] =  { -1,    dgtl5, -1 };
tSensors frontSonar[NUM_ROBOTS] = { dgtl6, -1,    -1 };
tSensors goalLine[NUM_ROBOTS] =   { in6,   in3,   in3 };
tSensors leftLine[NUM_ROBOTS] =   { -1,    -1,    -1 };
tSensors rightLine[NUM_ROBOTS] =  { -1,    -1,    -1 };
tSensors backLine[NUM_ROBOTS] =   { -1,    -1,    -1 };
	//#endsubregion
//#endregion

//#region consts
int sideSwitchPos[NUM_ROBOTS] = { 1845, 1960, 1960 };
//#endregion

//#region measurements
float liftLen[NUM_ROBOTS]        = { 14.75, 16,  16 };
float coneHeight[NUM_ROBOTS]     = { 3.5,   3.5, 3.5 };
float l_offset[NUM_ROBOTS]       = { 3.5,   4,   4 };
float goalToMidDist[NUM_ROBOTS]  = { 17,    18,  18 };	//distance from field diagonal to mid goal
float lineToGoalDist[NUM_ROBOTS] = { 26,    22,  22 };	//distance from line to mid goal - TODO: wtf?
float barToLineDist[NUM_ROBOTS]  = { 9,     9,   9 };
//#endregion

//#region cone counts
int apathyCones[NUM_ROBOTS] = { 0,  0,  0 };
int maxNumCones[NUM_ROBOTS] = { 16, 16, 16 };
//#endregion

//#region timing
int fbMoveDuration[NUM_ROBOTS]  = { 700, 700, 400 };
int outtakeDuration[NUM_ROBOTS] = { 250, 300, 300 };
int intakeDuration[NUM_ROBOTS]  = { -1,  300, 300 };
//#endregion

//#region buttons
	//#subregion variable buttons
tVexJoysticks fbInBtn[NUM_ROBOTS]     = { Btn6U, Btn8D, -1 };
tVexJoysticks fbOutBtn[NUM_ROBOTS]    = { Btn6D, Btn8U, -1 };
tVexJoysticks stackBtn[NUM_ROBOTS]    = { Btn8U, Btn6D, Btn6D };
tVexJoysticks defPosBtn[NUM_ROBOTS]   = { Btn8D, Btn8R, Btn8R };
tVexJoysticks maxPosBtn[NUM_ROBOTS]   = { Btn8L, -1,    -1 };
tVexJoysticks intakeBtn[NUM_ROBOTS]   = { -1,    Btn6U, -1 };
tVexJoysticks outtakeBtn[NUM_ROBOTS]  = { -1,    Btn6D, -1 };
tVexJoysticks toggleFbBtn[NUM_ROBOTS] = { -1,    Btn8L, Btn8D };
tVexJoysticks cycleInBtn[NUM_ROBOTS]  = { -1,    -1,    Btn6U };
tVexJoysticks cycleOutBtn[NUM_ROBOTS] = { -1,    -1,    Btn6D };
	//#endsubregion

	//#subregion common buttons
#define abortManeuversBtn Btn7L
#define shiftBtn          Btn7R
#define sayConeNumberBtn  Btn8L	//with shift

		//#subsubregion goal intake
#define goalIntakeBtn     Btn7D
#define goalOuttakeBtn    Btn7U
		//#endsubregion

		//#subsubregion lift
#define liftUpBtn         Btn5U
#define liftDownBtn       Btn5D
		//#endsubsubregion

		//#subsubregion autostacking control
#define toggleFieldingBtn Btn7D
#define resetConesBtn     Btn8R	//all cone count adjustment with shift
#define increaseConesBtn  Btn8U
#define decreaseConesBtn  Btn8D
		//#endsubsubregion
	//#endsubregion
//#endregion


//#region E Team
#ifdef E_TEAM_PASSIVE
	#include "E_PassivePragmas.c"

	#define PASSIVE true

	//#subregion positions
	/*enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 565,   565,     850,    565,        1180,   1200,   2050 };

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

		//#subregion consts
		#define SIDE_SWITCH_POS 1845	//middle of sidePos
		//#endsubregion

	//#subregion cone counts
	#define APATHY_CONES  0 //number of cones for which lift does not move
	#define MAX_NUM_CONES 16
	//#endsubregion

	//#subregion timing
	#define FB_MOVE_DURATION 700
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

	#define ROLLER true

	//#subregion positions
	/*enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
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

	//#subregion consts
	#define SIDE_SWITCH_POS  1960	//middle of sidePos
	#define OUTTAKE_DURATION 300
	#define INTAKE_DURATION  600
	//#endsubregion

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

	//#subregion timing
	#define FB_MOVE_DURATION 700
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
	//#endsubregion*/
#endif

#ifdef E_TEAM_PNEUMATIC
	#include "E_PneumaticPragmas.c"

	#define PNEUMATIC true

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1310,  1310,    1670,   1330,       2500,   1900,   2910 };

	enum fbState  { FB_FIELD, FB_SAFE, STACK, FB_MAX, FB_DEF };
	int fbPos[] = { 0,        0,       0,     0 };

	enum goalState  { OUT,  MID,  IN };
	int goalPos[] = { 3200, 2855, 1050 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 4
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port4, port5, port6, port7 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_LEFT_MOTORS 3
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port1, port3, port2 };

	#define NUM_RIGHT_MOTORS 3
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port8, port10, port9 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port2, port9 };
	//#endsubregion

	//#subregion pneumatics
	#define NUM_FB_SOLS 1
	tSensors fourBarSols[NUM_FB_SOLS] = { dgtl5 };

	#define NUM_INTAKE_SOLS 1
	tSensors intakeSols[NUM_INTAKE_SOLS] = { dgtl6 };
	//#endsubregion

	//#subregion sensors
	#define L_SENS_REVERSED  false	//lift
	#define FB_SENS_REVERSED false	//four bar
	#define L_ENC_REVERSED   true	//drive
	#define R_ENC_REVERSED   true

	#define HYRO          -1
	#define SIDE_POT      -1
	#define MODE_POT      -1
	#define LIFT_SENSOR   in1
	#define GOAL_SENSOR   in2
	#define GOAL_FOLLOWER in3
	#define ROLLER_ENC    -1
	#define LEFT_ENC      dgtl3
	#define RIGHT_ENC     dgtl1
	#define FRONT_SONAR   -1
	#define CONE_SONAR    -1

	#define LEFT_LINE     -1	//not currently attached
	#define BACK_LINE     -1
	#define RIGHT_LINE    -1
	#define FB_SENSOR     -1
	//#endsubregion

	//#subregion consts
	#define SIDE_SWITCH_POS  1960	//middle of sidePos
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN          16
	#define CONE_HEIGHT       3.5
	#define L_OFFSET          4
	#define GOAL_TO_MID_DIST  18
	#define LINE_TO_GOAL_DIST 22
	#define BAR_TO_LINE_DIST  9
	//#endsubregion

	//#subregion cone counts
	#define APATHY_CONES  0 //number of cones for which lift does not move
	#define MAX_NUM_CONES 16
	//#endsubregion

	//#subregion timing
	#define FB_MOVE_DURATION 400
	#define OUTTAKE_DURATION 300
	#define INTAKE_DURATION  600
	//#endsubregion

	//#subregion specific buttons
		//#subsubregion stack cycling
	#define cycleInBtn      Btn6U
	#define cycleOutBtn     Btn6D
		//#endsubsubregion

		//#subsubregion toggles
	#define toggleFbBtn     Btn8D
	#define toggleIntakeBtn Btn8U
		//#endsubsubregion

		//#subsubregion autostacking control
	#define stackBtn        Btn6D
		//#endsubsubregion

		//#subregion autopositioning
	#define defPosBtn       Btn8R
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
#define resetConesBtn     Btn8R
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
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1250
	//#endsubregion
//#endregion

//#region struct declaration
motorGroup goalIntake;
motorGroup lift;

#ifdef PASSIVE
	pneumaticGroup fourBar;
	pneumaticGroup intake;
#else
	motorGroup fourBar;
#endif

#ifdef ROLLER
	motorGroup roller;
#endif
//#endregion

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
	#ifdef PNEUMATIC
		initializePneumaticGroup();
	#else
		initializeGroup(fourBar, NUM_FB_MOTORS, fourBarMotors);
		configureButtonInput(fourBar, fbOutBtn, fbInBtn);
		configureBtnDependentStillSpeed(fourBar, FB_STILL_SPEED);

		if (FB_SENSOR >= 0) {
			initializeTargetingPID(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 13*FB_CORR_FCTR, 100/FB_CORR_FCTR);
			addSensor(fourBar, FB_SENSOR, FB_SENS_REVERSED);
			if (FB_SENSOR>=dgtl1) configureEncoderCorrection(fourBar, fbPos[FB_MAX]);
		}
	#endif

	#ifdef ROLLER
		initializeGroup(roller, NUM_ROLLER_MOTORS, rollerMotors);
		if (SKILLZ_MODE)
			configureButtonInput(roller, goalOuttakeBtn, goalIntakeBtn, ROLLER_STILL_SPEED);
		else
			configureButtonInput(roller, intakeBtn, outtakeBtn, ROLLER_STILL_SPEED);
	#endif
}
