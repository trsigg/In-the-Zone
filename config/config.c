#define E_TEAM


//#region options
#define HOLD_LAST_CONE true
#define SKILLZ_MODE    false
#define MULTIPLE_PIDs  false //if lift uses different PID consts for movement in different locations or directions
#define HAS_SPEAKER    true

	//#subregion testing - TODO: change parameter scheme
#define TESTING 0	//0 for normal behavior, 1 & 2 for PID testing (1 uses automatic still speeding, 2 uses only PID)
int debugParameters[] = { 0, 7 };	//{ liftDebugStartCol, liftSensorCol }
	//#endsubregion
//#endregion

//#region E Team
#ifdef E_TEAM
	#include "E_TeamPragmas.c"

	//#subregion positions
	enum liftState  { L_MIN, L_FIELD, L_SAFE, M_BASE_POS, D_LOAD, L_ZERO, L_MAX, L_DEF };	//when lift is at L_SAFE, goal intake can be moved without collision
	int liftPos[] = { 1425,  1430,    1880,   1420,       1880,   1880,   2960 };
	//#endsubregion

	//#subregion motors
	#define NUM_LIFT_MOTORS 2
	tMotor liftMotors[NUM_LIFT_MOTORS] = { port5, port7 };  //ROBOTC PRAGMAS! YOU DROVE ME TO DO THIS!

	#define NUM_RIGHT_MOTORS 2
	tMotor rightMotors[NUM_RIGHT_MOTORS] = { port1, port2 };

	#define NUM_LEFT_MOTORS 1
	tMotor leftMotors[NUM_LEFT_MOTORS] = { port6 };

	#define NUM_GOAL_MOTORS 2
	tMotor goalMotors[NUM_GOAL_MOTORS] = { port9, port10 };
	//#endsubregion

	//#subregion solenoids
	#define INTAKE_SOL dgtl5
	#define FB_SOL     dgtl6
	//#endsubregion

	//#subregion sensors
	#define L_USING_ENC     false //lift
	#define L_SENS_REVERSED false
	#define L_ENC_REVERSED  true	//drive
	#define R_ENC_REVERSED  true

	#define SIDE_POT     in1
	#define LIFT_SENSOR  in2
	#define LEFT_LINE    in4
	#define BACK_LINE    in5
	#define RIGHT_LINE   in6
	#define HYRO         in7
	#define LEFT_ENC     dgtl1
	#define RIGHT_ENC    dgtl3
	//#endsubregion

	//#subregion measurements
	#define LIFT_LEN 14.0
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

	//#subregion cone intake
#define toggleIntakeBtn   Btn8U	//toggles cone intake solenoid
	//#endsubregion

	//#subregion top four bar
#define toggle4bBtn       Btn6D
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

	//#subregion lift
#define liftUpBtn         Btn5U	//fielding mode
#define liftDownBtn       Btn5D
	//#endsubregion
//#endregion

//#region constants
	//#subregion sensor consts
#define RAD_TO_POT   880.1    //conversion factor between radians and potentiometer values
#define L_GEAR_RATIO 5	//gear ratio between lift bar angle and sensors
#define RAD_TO_ENC   (180 / PI) //conversion factor between radians and encoder values
const float RAD_TO_LIFT = (L_USING_ENC ? RAD_TO_ENC*L_GEAR_RATIO : RAD_TO_POT);
const float L_CORR_FCTR = (L_USING_ENC ? RAD_TO_POT/RAD_TO_LIFT : 1);
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
#define LIFT_STILL_SPEED   15
#define L_AUTO_SS_MARGIN   50
#define GOAL_STILL_SPEED   15
	//#endsubregion
	//#subregion cone counts
#define APATHY_CONES       0 //number of cones for which lift does not move
#define NO_OFFSET_CONES    1 //number of cones for which the lift goes straight to liftAngle2
#define D_LIFT_EARLY_CONES 3 //number of cones in driver load mode for which lift and chain go to defaults simultaneously (TODO: chain?)
#define MAX_NUM_CONES      15
	//#endsubregion
	//#subregion timing
#define OUTTAKE_DURATION      100
#define GOAL_INTAKE_DURATION  1500
#define GOAL_OUTTAKE_DURATION 1750
	//#endsubregion
//#endregion


#include "..\lib\pd_autoMove.c" //for drive declaration
#include "..\lib\pneumaticGroup.c"

motorGroup goalIntake;
motorGroup lift;
pneumaticGroup fourBar;
pneumaticGroup coneIntake;

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

	//mobile goal intake
	initializeGroup(goalIntake, NUM_GOAL_MOTORS, goalMotors);
	configureButtonInput(goalIntake, goalOuttakeBtn, goalIntakeBtn);
	configureBtnDependentStillSpeed(goalIntake, GOAL_STILL_SPEED);

	//cone intake
	initializePneumaticGroup(coneIntake, INTAKE_SOL);
	configureToggleInput(coneIntake, toggleIntakeBtn);

	//top four bar
	initializePneumaticGroup(fourBar, FB_SOL);
	configureToggleInput(fourBar, toggle4bBtn);
}
