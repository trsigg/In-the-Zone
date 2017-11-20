#define M_TEAM


//#region options
#define HOLD_LAST_CONE true
#define SKILLZ_MODE    false
#define MULTIPLE_PIDs  false //if chain bar and lift use different PID consts for movement in different locations or directions
#define HAS_SPEAKER    true

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
int debugParameters[] = { 0, -1, 7, -1 };	//{ liftDebugStartCol, chainDebugStartCol, liftSensorCol, chainSensorCol }
	//#endsubregion
//#endregion

//#region E Team
#ifdef E_TEAM
	#include "E_TeamPragmas.c"

	//#subregion positions
	enum chainState  { CH_FIELD, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
	int chainPos[] = { 146,      146,     42,    0,      42,   197 };

	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1425,  1430,    1880,   1420,       1880,   1880,   2960 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 2
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port5, port7 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_CHAIN_MOTORS 2
	tMotor chainMotors[NUM_CHAIN_MOTORS] = { port3, port8 };

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port1, port2 };

	#define NUM_LEFT_MOTORS 1
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port6 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port9, port10 };

	#define NUM_INTAKE_MOTORS 1
	tMotor intakeMotors[NUM_INTAKE_MOTORS] = { port4 };
	//#endsubregion

	//#subregion sensors
	#define L_USING_ENC      false //lift
	#define L_SENS_REVERSED  false
	#define CH_USING_ENC     true  //chain bar
	#define CH_SENS_REVERSED false
	#define L_ENC_REVERSED   true	//drive
	#define R_ENC_REVERSED   true

	#define SIDE_POT     in1
	#define LIFT_SENSOR  in2
	#define LEFT_LINE    in4
	#define BACK_LINE    in5
	#define RIGHT_LINE   in6
	#define HYRO         in7
	#define LEFT_ENC     dgtl1
	#define RIGHT_ENC    dgtl3
	#define CHAIN_SENSOR dgtl7
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 14.0
	//#endsubregion
#endif
//#endregion

//#region M TEAM
#ifdef M_TEAM
	#include "M_TeamPragmas.c"

	//#subregion options
	#undef  HAS_SPEAKER
	#define HAS_SPEAKER false
	//#endsubregion

	//#subregion positions
	enum chainState  { CH_FIELD, CH_SAFE, STACK, CH_MIN, VERT, CH_MAX, CH_DEF };  //when chain bar is at CH_SAFE, lift can move up and down without colliding with cone stack
	int chainPos[] = { 2745,     2070,    1060,  315,    1275, 3650 };

	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1180,  1190,    1465,   1170,       1625,   1625,   2410 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 2
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port2, port3 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_CHAIN_MOTORS 2
	tMotor chainMotors[NUM_CHAIN_MOTORS] = { port6, port7 };

	#define NUM_RIGHT_MOTORS 1
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port8 };

	#define NUM_LEFT_MOTORS 1
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port9 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port4, port5 };

	#define NUM_INTAKE_MOTORS 1
	tMotor intakeMotors[NUM_INTAKE_MOTORS] = { port1 };
	//#endsubregion

	//#subregion sensors
	#define L_USING_ENC      false //lift
	#define L_SENS_REVERSED  true
	#define CH_USING_ENC     false  //chain bar
	#define CH_SENS_REVERSED false
	#define L_ENC_REVERSED   false	//drive
	#define R_ENC_REVERSED   false

	#define LIFT_SENSOR  in1
	#define CHAIN_SENSOR in2
	#define SIDE_POT     in3
	#define HYRO         in4
	#define LEFT_ENC     dgtl1
	#define RIGHT_ENC    dgtl3

	#define LEFT_LINE    in7	//don't exist
	#define BACK_LINE    in5
	#define RIGHT_LINE   in6
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 12.5
	//#endsubregion
#endif
//#endregion

//#region buttons
#define abortManeuversBtn Btn7L
#define shiftBtn					Btn7R
#define sayConeNumberBtn  Btn8L	//with shift

	//#subregion goal intake
#define goalIntakeBtn			Btn7D
#define goalOuttakeBtn		Btn7U
	//#endsubregion

		//#subsubregion cone intake
#define coneIntakeBtn     Btn6U	//intakes alone, outtakes with shift
		//#endsubsubregion

	//#subregion autopositioning
#define chainDefBtn				Btn8D	//takes chain bar to default position
#define chainStackBtn			Btn8L //takes chain bar to STACK
                                //when pressed together, they take lift to L_DEF and chain bar to default position for mode
//#define maxPosBtn         Btn7U //takes lift to L_MAX
	//#endsubregion

	//#subregion autostacking control
#define stackBtn					Btn6D
#define toggleFieldingBtn Btn7D
		//#subsubregion cone count adjustment
#define resetBtn					Btn8R	//with shift
#define increaseConesBtn	Btn8U	//with shift
#define decreaseConesBtn	Btn8D	//with shift
		//#endsubsubregion
	//#endsubregion

//#subregion manual control
#define resetEncodersBtn  Btn7U
		//#subsubregion lift
#define f_liftUpBtn				Btn8U	//fielding mode
#define f_liftDownBtn			Btn8R
#define d_liftUpBtn       Btn5U	//driver load mode
#define d_liftDownBtn     Btn5D
		//#endsubsubregion
		//#subsubregion chain bar
#define f_chainInBtn			Btn5U	//fielding mode
#define f_chainOutBtn			Btn5D
#define d_chainInBtn			Btn8R	//driver load mode
#define d_chainOutBtn			Btn8U
		//#endsubsubregion
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT   880.1    //conversion factor between radians and potentiometer values
#define L_GEAR_RATIO 5	//gear ratio between lift bar angle and sensors
#define RAD_TO_ENC   (180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT = (L_USING_ENC ? RAD_TO_ENC*L_GEAR_RATIO : RAD_TO_POT);
const float L_CORR_FCTR = (L_USING_ENC ? RAD_TO_POT/RAD_TO_ENC/L_GEAR_RATIO : 1);
const float CH_CORR_FCTR = (CH_USING_ENC ? RAD_TO_POT/RAD_TO_ENC : 1);
#define R_LINE_THRESHOLD 2960
#define L_LINE_THRESHOLD 3060
#define B_LINE_THRESHOLD 2870
	//#endsubregion
	//#subregion measurements
#define CONE_HEIGHT 2.75
#define LIFT_OFFSET 2.0
#define GOAL_TO_MID_DIST 17.5
	//#endsubregion
	//#subregion still speeds
#define INTAKE_STILL_SPEED 15
#define LIFT_STILL_SPEED   15
#define L_AUTO_SS_MARGIN   50
#define CHAIN_STILL_SPEED  15
#define CH_AUTO_SS_MARGIN	 0
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES       0 //number of cones for which lift does not move
#define RECKLESS_CONES     1 //number of cones for which chain bar goes directly to STACK (not CH_SAFE first)
#define NO_OFFSET_CONES    1 //number of cones for which the lift goes straight to liftAngle2
#define D_LIFT_EARLY_CONES 3 //number of cones in driver load mode for which lift and chain go to defaults simultaneously
#define MAX_NUM_CONES      15
	//#endsubregion
	//#subregion timing
#define INTAKE_DURATION  400	//amount of time rollers activate when intaking/expelling cones
#define OUTTAKE_DURATION 450
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1750
	//#endsubregion
//#endregion

//#region timers
#define GOAL_TIMER T1
//#endregion


#include "..\lib\pd_autoMove.c" //for drive declaration

motorGroup goalIntake;
motorGroup coneIntake;
motorGroup lift;
motorGroup chainBar;

void initializeStructs() {
  //drive
	initializeDrive(drive, NUM_LEFT_MOTORS, leftMotors, NUM_RIGHT_MOTORS, rightMotors, true);
	attachEncoder(drive, LEFT_ENC, LEFT, L_ENC_REVERSED);
	attachEncoder(drive, RIGHT_ENC, RIGHT, R_ENC_REVERSED, 4.0, 2.0);
	attachGyro(drive, HYRO);

	//lift
  initializeGroup(lift, NUM_LIFT_MOTORS, liftMotors);
	initializeTargetingPID(lift, 0.4*L_CORR_FCTR, 0.005*L_CORR_FCTR, 5*L_CORR_FCTR, 10);	//gain setup in setLiftPIDmode when MULTIPLE_PIDs is true
	addSensor(lift, LIFT_SENSOR, L_SENS_REVERSED);
	if (L_USING_ENC) configureEncoderCorrection(lift, liftPos[L_MAX]);

	//chain bar
	initializeGroup(chainBar, NUM_CHAIN_MOTORS, chainMotors);
	initializeTargetingPID(chainBar, 0.46*CH_CORR_FCTR, 0.0001*CH_CORR_FCTR, 1.3*CH_CORR_FCTR, 10);	//gain setup in setChainBarPIDmode when MULTIPLE_PIDs is true
	addSensor(chainBar, CHAIN_SENSOR, CH_SENS_REVERSED);
	if (CH_USING_ENC) configureEncoderCorrection(chainBar, chainPos[CH_MAX]);

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);

	//cone intake
	initializeGroup(coneIntake, NUM_INTAKE_MOTORS, intakeMotors);
}
