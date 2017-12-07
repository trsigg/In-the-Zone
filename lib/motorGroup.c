#define numTargets 4

#include "coreIncludes.c"
#include "PID.c"
#include "timer.c"

enum controlType { NONE, BUTTON, JOYSTICK };
enum automovementType { NO, PID, MANEUVER, DURATION };

typedef struct {
	tMotor motors[12];
	int numMotors;
	controlType controlType;
	bool controlActive;
	TVexJoysticks posInput, negInput; //inputs. NegInput only assigned if using button control
	//button control
	int upPower, downPower, stillSpeed;
		//complex still speeds
	int stillSpeedSwitchPos, stillSpeedType;	//stillSpeedType is 0 for regular, 1 for posDependent, and 2 for buttonDependent
	bool stillSpeedReversed;
	//joystick control
	int deadband; //range of motor values around 0 for which motors are not engaged
	bool isRamped; //whether group is ramped
	int msPerPowerChange; //if ramping, time between motor power changes, calculated using maxAcc100ms
	float powMap; //degree of polynomial to which inputs are mapped (1 for linear)
	float coeff; //factor by which motor powers are multiplied
	long lastUpdated; //ramping
	//absolutes
	int absMin, absMax; //extreme  positions of motorGroup
	bool hasAbsMin, hasAbsMax;
	int maxPowerAtAbs, defPowerAtAbs; //maximum power at absolute position (pushing down from minimum or up from maximum) and default power if this is exceeded
	//automovement
	automovementType moving;
	int movePower, endPower;	//both maneuver and duration
	long maneuverTimer;
		//execute maneuver
	int targetPos, maneuverTimeout;
	bool forward; //forward: whether target is forwad from initial group position
		//move for duration
	int moveDuration;
		//maintainPos
	PID posPID;	//PID controller which maintains position
		//sensors
	bool hasEncoder, hasPotentiometer;
	bool encoderReversed, potentiometerReversed;
	bool potentiometerDefault; //whether potentiometer (as opposed to encoder) is default sensor for position measurements
	bool encCorrectionActive;	//drift correction
	int encMax, maxDisp;
	tSensors encoder, potentiometer;
} motorGroup;

//#region initialization
void configureButtonInput(motorGroup *group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->controlType = BUTTON;
	group->controlActive = true;
	group->posInput = posBtn;
	group->negInput = negBtn;
	group->stillSpeed = stillSpeed;
	group->stillSpeedType = 0;
	group->stillSpeedReversed = false;
	group->upPower = upPower;
	group->downPower = downPower;
}

void configureJoystickInput(motorGroup *group, TVexJoysticks joystick, int deadband=10, bool isRamped=false, int maxAcc100ms=60, float powMap=1, int maxPow=127) {
	group->controlType = JOYSTICK;
	group->controlActive = true;
	group->posInput = joystick;
	group->deadband = deadband;
	group->isRamped = isRamped;
	group->msPerPowerChange = 100 / maxAcc100ms;
	group->powMap = powMap;
	group->coeff = maxPow /  127.0;
	group->lastUpdated = nPgmTime;
}

void configureRamping(motorGroup *group, int maxAcc100ms) {
	group->isRamped = true;
	group->msPerPowerChange = 100 / maxAcc100ms;
}

void initializeGroup(motorGroup *group, int numMotors, tMotor *motors, TVexJoysticks posBtn=Ch1, TVexJoysticks negBtn=Ch1, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->numMotors = limit(numMotors, 0, 12);

	for (int i=0; i<group->numMotors; i++)
		group->motors[i] = motors[i];

	if (posBtn >= Btn5D)
		configureButtonInput(group, posBtn, negBtn, stillSpeed, upPower, downPower);

	group->maneuverExecuting = false;
}

void configurePosDependentStillSpeed(motorGroup *group, int switchPos, int stillSpeed=0) {	//motor will have stillSpeed power when below switchPos, -stillSpeed power when above switchPos
	if (stillSpeed!=0) group->stillSpeed = stillSpeed;
	group->stillSpeedType = 1;
	group->stillSpeedSwitchPos = switchPos;
}

void configureBtnDependentStillSpeed(motorGroup *group, int stillSpeed=0) {
	if (stillSpeed!=0) group->stillSpeed = stillSpeed;
	group->stillSpeedType = 2;
}
//#endregion

//#region sensors
void addSensor(motorGroup *group, tSensors sensor, bool reversed=false, bool setAsDefault=true) {
	switch (SensorType[sensor]) {
		case sensorPotentiometer:
			group->hasPotentiometer = true;
			group->potentiometer = sensor;
			group->potentiometerReversed = reversed;
			if (setAsDefault) group->potentiometerDefault = true;
			break;
		case sensorQuadEncoder:
			group->hasEncoder = true;
			group->encoder = sensor;
			group->encoderReversed = reversed;
			SensorValue[sensor] = 0;
			if (setAsDefault) group->potentiometerDefault = false;
			break;
	}
}

int encoderVal(motorGroup *group) {
	if (group->hasEncoder) {
		return SensorValue[group->encoder] * (group->encoderReversed ?  -1 : 1);
	} else {
		return 0;
	}
}

int potentiometerVal(motorGroup *group) {
	if (group->hasPotentiometer) {
		return (group->potentiometerReversed ? 4096-SensorValue[group->potentiometer] : SensorValue[group->potentiometer]);
	} else {
		return 0;
	}
}

int getPosition(motorGroup *group) {
	if (group->hasPotentiometer && group->hasEncoder) {
		return group->potentiometerDefault ? potentiometerVal(group) : encoderVal(group);
	} else {
		return (group->hasEncoder ? encoderVal(group) : potentiometerVal(group));
	}
}

void resetEncoder(motorGroup *group, int resetVal=0) {
	if (group->hasEncoder)
		SensorValue[group->encoder] = resetVal * (group->encoderReversed ?  -1 : 1);
}

void configureEncoderCorrection(motorGroup *group, int max) {
	group->encMax = max;
	group->encCorrectionActive = true;
	group->maxDisp = 0;
}

void correctEncVal(motorGroup *group) {
	if (group->encCorrectionActive) {
		int encVal = encoderVal(group);

		if (encVal < group->maxDisp) {
			resetEncoder(group);
			group->maxDisp = 0;
		}
		else if ((encVal - group->encMax) > group->maxDisp) {
			group->maxDisp = encVal - group->encMax;
		}
	}
}
//#endregion

//#region position limiting
void setAbsMax(motorGroup *group, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMax = max;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsMin(motorGroup *group, int min, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->hasAbsMin = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->hasAbsMin = true;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region set and get power
int setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {
	if (!overrideAbsolutes) {
		if (group->hasAbsMin && getPosition(group) <= group->absMin && power < -group->maxPowerAtAbs)
			power = -group->defPowerAtAbs;

		if (group->hasAbsMax && getPosition(group) >= group->absMax && power > group->maxPowerAtAbs)
			power = group->defPowerAtAbs;
	}

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

	return power;
}

int getPower(motorGroup *group) {
	return group->motors[0];
}
//#endregion

//#region position movement
	//#subregion maintainPos
	void initializeTargetingPID(motorGroup *group, float kP, float kI, float kD, int minSampleTime=25, int integralMax=127) {
		initializePID(group->posPID, 0, kP, kI, kD, minSampleTime, integralMax);
	}

	void setTargetingPIDconsts(motorGroup *group, float kP, float kI, float kD) {
		changeGains(group->posPID, kP, kI, kD);
	}

	void setTargetPosition(motorGroup *group, int position, bool resetIntegral=true) {
		changeTarget(group->posPID, position, resetIntegral);
		group->moving = PID;
	}

	void maintainTargetPos(motorGroup *group, int debugStartCol=-1) {
		if (group->moving==PID && group->posPID.kP!=0) {
			setPower(group, PID_runtime(group->posPID, getPosition(group), debugStartCol));
		}
	}

	void stopTargeting(motorGroup *group) {
		if(group->moving == PID)
			group->moving = NO;
	}

	bool errorLessThan(motorGroup *group, int errorMargin) {	//returns true if PID error is less than specified margin
		return fabs(group->posPID.target - getPosition(group)) < errorMargin;
	}
	//#endsubregion

int moveTowardPosition(motorGroup *group, int position, int power=127) {
	return setPower(group, power * sgn(position - getPosition(group)));
}

	//#subregion maneuvers
void executeManeuver(motorGroup *group) {
	if (group->moving = MANEUVER) {
		if (group->forward == (getPosition(group) < group->targetPos)) {
			group->maneuverTimer = resetTimer();
			setPower(group, group->movePower);
		}

		if (time(group->maneuverTimer) > group->maneuverTimeout) {
			group->moving = NO;
			setPower(group, group->endPower);
		}
	}
}

void createManeuver(motorGroup *group, int position, int endPower=0, int movePower=127, int timeout=10) {
	group->targetPos = position;
	group->endPower = endPower;
	group->forward = group->targetPos > getPosition(group);
	group->movePower = abs(movePower) * (group->forward ? 1 : -1);
	group->maneuverTimeout = timeout;
	group->maneuverTimer = resetTimer();
	group->moving = MANEUVER;

	setPower(group, movePower);
}

void goToPosition(motorGroup *group, int position, bool runConcurrently=false, int endPower=0, int movePower=127, int timeout=100) {
	long posTimer = resetTimer();
	int displacementSign = sgn(position - getPosition(group));
	setPower(group, displacementSign*movePower);

	while (time(posTimer) < timeout) {
		if (sgn(position - getPosition(group)) == displacementSign) posTimer = resetTimer();
	}

	setPower(group, endPower);
}
	//#endsubregion

void moveForDuration(motorGroup *group, int power, int duration, int endPower=0, bool runAsTask=false) {
	group->movePower = power;
	group->moveDuration = duration;
	group->endPower = endPower;
}
//#endregion

//#region user input
int handleButtonInput(motorGroup *group) {
	if (vexRT[group->posInput] == 1) {
		group->maneuverExecuting = false;
		group->activelyMaintining = false;

		if (group->stillSpeedType == 2)
			group->stillSpeedReversed = false;

		return group->upPower;
	} else if (vexRT[group->negInput] == 1) {
		group->maneuverExecuting = false;
		group->activelyMaintining = false;

		if (group->stillSpeedType == 2)
			group->stillSpeedReversed = true;

		return group->downPower;
	} else {
		if (group->stillSpeedType == 1)
			group->stillSpeedReversed = getPosition(group) > group->stillSpeedSwitchPos;

		executeManeuver(group);

		if (group->maneuverExecuting)
			return group->movePower;
		else
			return group->stillSpeed * (group->stillSpeedReversed ? -1 : 1);
	}
}

int handleJoystickInput(motorGroup *group) {
	int input = vexRT[group->posInput];
	int power = sgn(input) * group->coeff * fabs(pow(input / 127.0, group->powMap)) * 127;

	if (abs(power) < group->deadband) power = 0;

	//handle ramping
	if (group->isRamped) {
		long now = nPgmTime;
		int elapsed = now - group->lastUpdated;
		int currentPower = motor[ group->motors[0] ];

		if (elapsed >= group->msPerPowerChange) {
			group->lastUpdated = now;

			if (abs(power) > abs(currentPower)) {	//only ramp up in absolute value
				int maxDiff = elapsed / group->msPerPowerChange;

				if (abs(currentPower - power) > maxDiff) {
					group->lastUpdated = now - (elapsed % group->msPerPowerChange);
					return (power>currentPower ? currentPower+maxDiff : currentPower-maxDiff);
				}
			}
		} else {
			return currentPower;
		}
	}

	return power;
}

int takeInput(motorGroup *group, bool setMotors=true) {
	int power = 0;

	if (group->controlActive) {
		switch (group->controlType) {
			case BUTTON:
				power = handleButtonInput(group);
				break;
			case JOYSTICK:
				power = handleJoystickInput(group);
				break;
		}
	}

	if (setMotors) setPower(group, power);

	return power;
}
//#endregion
