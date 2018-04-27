#include "..\lib\pd_autoMove.c"
#include "..\lib\pneumaticGroup.c"


enum robotId { E_PASSIVE, E_ROLLER, E_PNEUMATIC };
#define NUM_ROBOTS 3	//number of configurable robots

//#define E_TEAM_PASSIVE
#define E_TEAM_ROLLER
//#define E_TEAM_PNEUMATIC
//#define RUN_AUTON_AS_MAIN


//#region options
//#define HAS_SPEAKER
#define MULTIPLE_PIDs    false //if lift uses different PID consts for movement in different locations or directions
#define HOLD_LAST_CONE   true	//if lift stays up after stacking last cone
#define DOUBLE_DRIVER    false
#define SONAR_STACKING   true
#define LIMIT_GOAL_MVMNT true //if goal won't move unless lifted up
#define AUTOSTACK_CONFIG false	//using autostacking-focused button config (currently nonfunctional) (TODO: change to shift reverse intake)
#define DUAL_DRIVER      true
#define SONAR_IN_SKILLZ  true

	//#subregion auton/skillz options
#define SKILLZ_MODE      false	//skills
#define SKILLZ_VARIANT   false
#define PARK_IN_SKILLS   false
#define CROSS_FIELD_SKLZ false
#define SKILLZ_5PT       false
#define RECKON_IN_SKILLZ false
#define PULSE_ROLLERS    true
#define ABORT_AFTER_15   true	//general
#define USE_ENC_CORR     true
#define DRIVE_FOR_10     false
#define ANTI_MARK        1
#define ABORT_IF_NO_GOAL false
#define RETRY_GOAL_FAILS false
#define MAX_GOAL_RETRIES 2
#define DEFENSIVE_DELAY  1000
	//#endsubregion

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID), 3 for misc testing
int debugParameters[] = { 0,       7,        -1,    -1,     -1,        -1,       -1,        -1,      -1,       -1,    -1,      -1,       -1,      -1 };
//                      { liftPID, liftSens, fbpid, fbSens, driveRamp, turnRamp, coneSonar, goalPot, liftPow, fbPow, leftPow, rightPow, goalPow, customPow }

#define NUM_DEBUG_PORTS 5
int debugPorts[NUM_DEBUG_PORTS] = { port1, port2, port3, port6, port9 };
	//#endsubregion
//#endregion


//#region E Team
#ifdef E_TEAM_PASSIVE
	#include "E_PassivePragmas.c"

	#define PASSIVE true
	robotId robot = E_PASSIVE;

	/*//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
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
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port3, port8 };*/
	//#endsubregion
#endif

#ifdef E_TEAM_ROLLER
	#include "E_RollerPragmas.c"

	#define ROLLER true
	robotId robot = E_ROLLER;

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, S_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1400,  1415,    1650,   1410,       2050,       2250,   1990,   3045 };

	enum fbState  { FB_UP, FB_SAFE, FB_DOWN };
	int fbPos[] = { 1500,  3000,    3550 };

	enum goalState  { OUT,  MID,  IN };
	int goalPos[] = { 3375, 2850, 1160 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 3
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port4, port5, port7 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_FB_MOTORS 1
	tMotor fourBarMotors[NUM_FB_MOTORS] = { port10 };

	#define NUM_LEFT_MOTORS 2
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port6, port2 };

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port3, port9 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port2, port9 };

	#define NUM_ROLLER_MOTORS 1
	tMotor rollerMotors[NUM_ROLLER_MOTORS] = { port1 };
	//#endsubregion
#endif

#ifdef E_TEAM_PNEUMATIC
	#include "E_PneumaticPragmas.c"

	#define PNEUMATIC true
	robotId robot = E_PNEUMATIC;

	//#subregion positions
	/*enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1245,  1275,    1560,   1300,       2020,   1860,   2900 };

	enum goalState  { OUT,  MID,  IN };
	int goalPos[] = { 3300, 2770, 920 };
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
	#define NUM_FB_SOLS 2
	tSensors fourBarSols[NUM_FB_SOLS] = { dgtl5, dgtl7 };

	#define NUM_INTAKE_SOLS 1
	tSensors intakeSols[NUM_INTAKE_SOLS] = { dgtl6 };

	#define NUM_BRAKE_SOLS 1
	tSensors brakeSols[NUM_BRAKE_SOLS] = { dgtl8 };*/
	//#endsubregion
#endif
//#endregion

//#region sensors
	//#subregion config
bool liftSensReversed[NUM_ROBOTS] = { false, false, false };
bool fbSensReversed[NUM_ROBOTS]   = { false, false, false };
bool goalSensReversed[NUM_ROBOTS] = { false, true,  false };
bool L_EncReversed[NUM_ROBOTS]    = { false, false, true };
bool R_EncReversed[NUM_ROBOTS]    = { true,  true, false };
	//#endsubregion

	//#subregion ports
tSensors hyro[NUM_ROBOTS]       = { in1,   in8,   in3 };
tSensors liftSensor[NUM_ROBOTS] = { in4,   in1,   in1 };
tSensors goalSensor[NUM_ROBOTS] = { in5,   in6,   in6 };
tSensors fbSensor[NUM_ROBOTS]   = { -1,    in7,   -1 };
tSensors sidePot[NUM_ROBOTS]    = { in2,   in5,   in5 };
tSensors modePot[NUM_ROBOTS]    = { in3,   in4,   in4 };
tSensors configPot[NUM_ROBOTS]  = { -1,    in3,   -1 };
tSensors leftEnc[NUM_ROBOTS]    = { dgtl1, dgtl3, dgtl3 };
tSensors rightEnc[NUM_ROBOTS]   = { dgtl3, dgtl1, dgtl1 };
tSensors coneSonar[NUM_ROBOTS]  = { -1,    dgtl5, -1 };
tSensors frontSonar[NUM_ROBOTS] = { dgtl6, -1,    -1 };
tSensors goalLine[NUM_ROBOTS]   = { in6,   in2,   in2 };
tSensors leftLine[NUM_ROBOTS]   = { -1,    -1,    -1 };
tSensors rightLine[NUM_ROBOTS]  = { -1,    -1,    -1 };
tSensors backLine[NUM_ROBOTS]   = { -1,    -1,    -1 };
	//#endsubregion
//#endregion

//#region consts
int sideSwitchPos[NUM_ROBOTS] = { 1845, 1910, 1910 };

	//#subregion sensor consts
int goalLineThresh[NUM_ROBOTS]  = { -1,   2990, 2960 };
int l_lineThresh[NUM_ROBOTS]    = { 3060, -1,   -1 };
int r_lineThresh[NUM_ROBOTS]    = { 2960, -1,   -1 };
int b_lineThresh[NUM_ROBOTS]    = { 2870, -1,   -1 };
int coneSonarThresh[NUM_ROBOTS] = { 1000, 1600, 2500 };
float liftGearRatio[NUM_ROBOTS] = { 5,    5,    5 };	//gear ratio between lift bar angle and sensors

#define RAD_TO_POT 880.1      //conversion factor between radians and potentiometer values
#define RAD_TO_ENC (180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT  = (liftSensor[robot]>=dgtl1 ? RAD_TO_ENC*liftGearRatio[robot] : RAD_TO_POT);
const float L_CORR_FCTR  = (liftSensor[robot]>=dgtl1 ? RAD_TO_POT/RAD_TO_LIFT : 1);
const float FB_CORR_FCTR = (fbSensor[robot]>=dgtl1 ? RAD_TO_POT/RAD_TO_ENC : 1);
	//#endsubregion
	//#subregion still speeds
int l_StillSpeed[NUM_ROBOTS]     = { 15, 15, 15 };
int l_AutoSSMargin[NUM_ROBOTS]   = { 50, -1, -1 };
int fbStillSpeed[NUM_ROBOTS]     = { 15, 20, -1 };
int fbAutoSSMargin[NUM_ROBOTS]   = { 50, -1, -1 };
int goalStillSpeed[NUM_ROBOTS]   = { 15, 15, 0 };
int rollerStillSpeed[NUM_ROBOTS] = { -1, 20, -1 };
	//#endsubregion
//#endregion

//#region measurements
float liftLen[NUM_ROBOTS]        = { 14.75, 15.5, 15.5 };
float coneHeight[NUM_ROBOTS]     = { 3.5,   3.25, 3.25 };
float l_offset[NUM_ROBOTS]       = { 3.5,   2.5,  5 };
float goalToMidDist[NUM_ROBOTS]  = { 17,    12,   17 };	//distance from field diagonal to mid goal
float lineToGoalDist[NUM_ROBOTS] = { 26,    10,   22 };	//distance from line to mid goal - TODO: wtf?
float barToLineDist[NUM_ROBOTS]  = { 9,     9,    9 };
float interConeDist[NUM_ROBOTS]  = { 9,     7.5,  9 };
//#endregion

//#region cone counts
int apathyCones[NUM_ROBOTS] = { 0,  0,  0 };
int maxNumCones[NUM_ROBOTS] = { 16, 16, 16 };
//#endregion

//#region timing
int fbMoveDuration[NUM_ROBOTS]  = { 700,  600,  200 };
int outtakeDuration[NUM_ROBOTS] = { 250,  100,  300 };
int intakeDuration[NUM_ROBOTS]  = { -1,   300,  300 };
int goalOutDuration[NUM_ROBOTS] = { 1500, 1500, 1500 };
int goalinDuration[NUM_ROBOTS]  = { 1250, 1250, 1250 };
//#endregion

//#region buttons
	//#subregion variable buttons
//c is for comp mode, s for skills
TVexJoysticks c_fbInBtn[NUM_ROBOTS]      = { Btn6U, Btn8D, -1 };
TVexJoysticks c_fbOutBtn[NUM_ROBOTS]     = { Btn6D, Btn8U, -1 };
TVexJoysticks s_fbInBtn[NUM_ROBOTS]      = { Btn7D, Btn8D, -1 };
TVexJoysticks s_fbOutBtn[NUM_ROBOTS]     = { Btn7U, Btn8U, -1 };
TVexJoysticks stackBtn[NUM_ROBOTS]       = { Btn8U, Btn8R, Btn8D };
TVexJoysticks safePosBtn[NUM_ROBOTS]     = { Btn8D, -1,    -1 };
TVexJoysticks maxPosBtn[NUM_ROBOTS]      = { Btn8L, -1,    -1 };
TVexJoysticks c_intakeBtn[NUM_ROBOTS]    = { -1,    Btn6U, Btn8R };
TVexJoysticks c_outtakeBtn[NUM_ROBOTS]   = { -1,    Btn6D, -1 };
TVexJoysticks s_intakeBtn[NUM_ROBOTS]    = { -1,    Btn7U, -1 };
TVexJoysticks s_outtakeBtn[NUM_ROBOTS]   = { -1,    Btn7D, -1 };
TVexJoysticks toggleFbBtn[NUM_ROBOTS]    = { -1,    Btn8L, Btn8L };
TVexJoysticks brakeBtn[NUM_ROBOTS]       = { -1,    -1,    Btn8U };
TVexJoysticks toggleFieldBtn[NUM_ROBOTS] = { Btn7D, Btn7D, Btn7D };
TVexJoysticks toggleCtrlBtn[NUM_ROBOTS]  = { -1,    Btn7R, -1 };
TVexJoysticks c_cycleInBtn[NUM_ROBOTS]   = { -1,    -1,    Btn6U };
TVexJoysticks c_cycleOutBtn[NUM_ROBOTS]  = { -1,    -1,    Btn6D };
TVexJoysticks s_cycleInBtn[NUM_ROBOTS]   = { -1,    -1,    Btn7U };
TVexJoysticks s_cycleOutBtn[NUM_ROBOTS]  = { -1,    -1,    Btn7D };
TVexJoysticks shiftBtn[NUM_ROBOTS]       = { Btn7R, -1,    Btn7R };
	//#endsubregion

	//#subregion common buttons
#define abortManeuversBtn Btn7L
#define sayConeNumberBtn  Btn8L	//with shift

		//#subsubregion goal intake
#define c_goalIntakeBtn    Btn7D
#define c_goalOuttakeBtn   Btn7U
#define s_goalIntakeBtn    Btn6U
#define s_goalOuttakeBtn   Btn6D
		//#endsubregion

		//#subsubregion lift
#define liftUpBtn         Btn5U
#define liftDownBtn       Btn5D
		//#endsubsubregion

		//#subsubregion autostacking control
#define resetConesBtn     Btn8R	//all cone count adjustment with shift
#define increaseConesBtn  Btn8U
#define decreaseConesBtn  Btn8D
		//#endsubsubregion
	//#endsubregion
//#endregion


//#region struct declaration
motorGroup goalIntake;
motorGroup lift;

#ifdef PNEUMATIC
	pneumaticGroup fourBar;
	pneumaticGroup intake;
	pneumaticGroup brakes;
#else
	motorGroup fourBar;
#endif

#ifdef ROLLER
	motorGroup roller;
#endif
//#endregion

float generalDebug[] = { 0, 0 };

//motorGroup groupWaitList[DEF_WAIT_LIST_LEN] = { lift, fourBar, goalIntake };

bool inSkillzControl;

void setControlMode(bool skillz) {
	if (skillz) {
		configureButtonInput(fourBar, s_fbOutBtn[robot], s_fbInBtn[robot]);
		configureButtonInput(goalIntake, s_goalOuttakeBtn, s_goalIntakeBtn);
		configureButtonInput(roller, s_intakeBtn[robot], s_outtakeBtn[robot], rollerStillSpeed[robot]);	//be careful, usercontrol()'s behavior depends on the order of these buttons
	}
	else {
		configureButtonInput(fourBar, c_fbOutBtn[robot], c_fbInBtn[robot]);
		configureButtonInput(goalIntake, c_goalOuttakeBtn, c_goalIntakeBtn);
		configureButtonInput(roller, c_intakeBtn[robot], c_outtakeBtn[robot], rollerStillSpeed[robot]);
	}

	configureBtnDependentStillSpeed(fourBar, fbStillSpeed[robot]);
	configureBtnDependentStillSpeed(goalIntake, (SKILLZ_MODE ? goalStillSpeed[robot] : 0));

	inSkillzControl = skillz;
}

void initializeStructs() {
	//arrayCopy(groupWaitList, defGroupWaitList, DEF_WAIT_LIST_LEN);

  //drive
	int skillzAdjustment = (SKILLZ_MODE ? 1 : 0);
	initializeDrive(drive, NUM_LEFT_MOTORS-skillzAdjustment, leftMotors, NUM_RIGHT_MOTORS-skillzAdjustment, rightMotors, true, (SKILLZ_MODE ? 30 : 40));
	if (DUAL_DRIVER) configureDualDriver(drive);
	attachEncoder(drive, leftEnc[robot], LEFT, L_EncReversed[robot]);
	attachEncoder(drive, rightEnc[robot], RIGHT, R_EncReversed[robot], 4);
	attachUltrasonic(drive, frontSonar[robot]);
	attachGyro(drive, hyro[robot]);
	initializeGyro(drive, 139, 1865);

	if (SKILLZ_MODE) {
		driveDefaults.rampConst1 = 15;
		driveDefaults.rampConst2 = 0.005;
		driveDefaults.rampConst3 = 20;
		driveDefaults.rampConst4 = 0.07;
		driveDefaults.rampConst5 = 150;

		turnDefaults.rampConst1 = 6;
		turnDefaults.rampConst2 = 0.02;
		turnDefaults.rampConst3 = 20;
		turnDefaults.rampConst4 = 0.05;
		turnDefaults.rampConst5 = 150;
	}

	//lift
  initializeGroup(lift, NUM_LIFT_MOTORS, liftMotors, liftUpBtn, liftDownBtn, l_StillSpeed[robot]);
	configureBtnDependentStillSpeed(lift);
	initializeTargetingPID(lift, 0.3*L_CORR_FCTR, 0.0001*L_CORR_FCTR, 30*L_CORR_FCTR, 100/L_CORR_FCTR);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	configureAutoStillSpeed(lift, l_AutoSSMargin[robot]);
	addSensor(lift, liftSensor[robot], liftSensReversed[robot]);
	if (liftSensor[robot]>=dgtl1) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);	//input setup handled in setControlMode()
	addSensor(goalIntake, goalSensor[robot], goalSensReversed[robot]);

	//top four bar
	#ifdef PNEUMATIC
		initializePneumaticGroup(fourBar, NUM_FB_SOLS, fourBarSols, fbMoveDuration[robot]);
		configureToggleInput(fourBar, toggleFbBtn[robot]);

		initializePneumaticGroup(intake, NUM_INTAKE_SOLS, intakeSols);
		configureToggleInput(intake, c_intakeBtn[robot]);

		initializePneumaticGroup(brakes, NUM_BRAKE_SOLS, brakeSols);
		configureToggleInput(brakes, brakeBtn[robot]);
	#else
		initializeGroup(fourBar, NUM_FB_MOTORS, fourBarMotors);

		if (fbSensor[robot] >= 0) {
			//initializeTargetingPID(fourBar, 0.46*FB_CORR_FCTR, 0.0001*FB_CORR_FCTR, 13*FB_CORR_FCTR, 100/FB_CORR_FCTR);
			addSensor(fourBar, fbSensor[robot], fbSensReversed[robot]);
			if (fbSensor[robot]>=dgtl1) configureEncoderCorrection(fourBar, fbPos[FB_UP]);
		}
	#endif

	//roller intake
	#ifdef ROLLER
		initializeGroup(roller, NUM_ROLLER_MOTORS, rollerMotors);
	#endif

	setControlMode(SKILLZ_MODE);
}
